#include "recast_build_tempobstacle.h"
#include "InputGeom.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCrowd.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "DetourNavMeshQuery.h"

#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "./fastlz.h"

#include <cstring>
#include <assert.h>
#include <float.h>

static bool isectSegAABB(const float* sp, const float* sq,
	const float* amin, const float* amax,
	float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;

	float d[3];
	rcVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)

						// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}

	return true;
}

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize * 4;
}


struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeom* m_geom;

	inline MeshProcess() : m_geom(0)
	{
	}

	inline void init(InputGeom* geom)
	{
		m_geom = geom;
	}

	void save(FILE* fp)
	{
		// nothing to save
		return;
		if (!fp)
			return;
		if (m_geom)
		{
			int ConCount = m_geom->getOffMeshConnectionCount();
			fwrite(&ConCount, sizeof(int), 1, fp);
			if (ConCount > 0)
			{
				fwrite(m_geom->getOffMeshConnectionVerts(), sizeof(float), ConCount * 3 * 2, fp);
				fwrite(m_geom->getOffMeshConnectionRads(), sizeof(float), ConCount, fp);
				fwrite(m_geom->getOffMeshConnectionDirs(), sizeof(char), ConCount, fp);
				fwrite(m_geom->getOffMeshConnectionAreas(), sizeof(char), ConCount, fp);
				fwrite(m_geom->getOffMeshConnectionFlags(), sizeof(int), ConCount, fp);
				fwrite(m_geom->getOffMeshConnectionId(), sizeof(int), ConCount, fp);
			}
		}
		else
		{
			int count0 = 0;
			fwrite(&count0, sizeof(int), 2, fp);
		}
	}

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
				polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
				polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
		if (m_geom)
		{
			params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
			params->offMeshConRad = m_geom->getOffMeshConnectionRads();
			params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
			params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
			params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
			params->offMeshConUserID = m_geom->getOffMeshConnectionId();
			params->offMeshConCount = m_geom->getOffMeshConnectionCount();
		}
	}
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		triareas(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
	}

	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete[] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}

	rcHeightfield* solid;
	unsigned char* triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	TileCacheData tiles[MAX_LAYERS];
	int ntiles;
};


RaycastBuildTempObstacle::RaycastBuildTempObstacle() :
	m_keepInterResults(false),
	m_tileCache(0),
	m_cacheBuildTimeMs(0),
	m_cacheCompressedSize(0),
	m_cacheRawSize(0),
	m_cacheLayerCount(0),
	m_cacheBuildMemUsage(0),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(48),
	m_tileBuildTime(0),
	m_tileMemUsage(0),
	m_tileTriCount(0)

{
	m_talloc = new LinearAllocator(1024000);
	m_tcomp = new FastLZCompressor;
	m_tmproc = new MeshProcess;
	resetCommonSettings();
}

RaycastBuildTempObstacle::~RaycastBuildTempObstacle()
{
	cleanup();
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	dtFreeTileCache(m_tileCache);
}

void RaycastBuildTempObstacle::cleanup()
{
}

int RaycastBuildTempObstacle::addMeshEnd()
{
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	m_maxTiles = 1 << tileBits;
	m_maxPolysPerTile = 1 << polyBits;
	return 0;
}

#ifdef UNITY_EDITOR
extern void NavMeshBuildStep(const char* log, int total, int step);
#endif

int RaycastBuildTempObstacle::build(bool genDrop)
{
	dtStatus status;

	if (!m_geom || !m_geom->meshInited())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return 1;
	}

	m_tmproc->init(m_geom);

	// Init cache
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Generation params.
	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);

	// Tile cache params.
	dtTileCacheParams tcparams;
	memset(&tcparams, 0, sizeof(tcparams));
	rcVcopy(tcparams.orig, bmin);
	tcparams.cs = m_cellSize;
	tcparams.ch = m_cellHeight;
	tcparams.width = (int)m_tileSize;
	tcparams.height = (int)m_tileSize;
	tcparams.walkableHeight = m_agentHeight;
	tcparams.walkableRadius = m_agentRadius;
	tcparams.walkableClimb = m_agentMaxClimb;
	tcparams.maxSimplificationError = m_edgeMaxError;
	tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = 5120;

	int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	m_maxTiles = 1 << tileBits;
	m_maxPolysPerTile = 1 << polyBits;

	dtFreeTileCache(m_tileCache);

	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return 2;
	}
	status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return 3;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return 4;
	}

	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	rcVcopy(params.orig, bmin);
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.widthTile = tw;
	params.heightTile = th;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	params.agentRadius = m_agentRadius;
	params.jumpDistHori = m_jumpDistHori;
	params.jumpSafeHeight = m_dropHeightAutoMove;
	params.climbHeight = m_agentMaxClimb;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return 5;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return 6;
	}

	// Preprocess tiles.
	m_ctx->resetTimers();

	m_cacheLayerCount = 0;
	m_cacheCompressedSize = 0;
	m_cacheRawSize = 0;

	int m_maxTrisPerChunk = 0;
	std::vector<MeshObj>& meshes = m_geom->getMeshObjs();
	for (int i = 0; i < meshes.size(); i++)
	{
		if (m_maxTrisPerChunk < meshes[i].chunkyMesh->maxTrisPerChunk)
		{
			m_maxTrisPerChunk = meshes[i].chunkyMesh->maxTrisPerChunk;
		}
	}

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
#ifdef UNITY_EDITOR
			NavMeshBuildStep("rasterizeTileLayers", th*tw, y*tw + x);
#endif
			TileCacheData tiles[MAX_LAYERS];
			memset(tiles, 0, sizeof(tiles));
			int ntiles = rasterizeTileLayers(genDrop, x, y, cfg, tiles, MAX_LAYERS);
			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}

				m_cacheLayerCount++;
				m_cacheCompressedSize += tile->dataSize;
				m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}

	// Build initial meshes
	m_ctx->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
		{
#ifdef UNITY_EDITOR
			NavMeshBuildStep("buildNavMeshTiles", th*tw, y*tw + x);
#endif
			m_tileCache->buildNavMeshTilesAt(x, y, m_navMesh);
		}
	m_ctx->stopTimer(RC_TIMER_TOTAL);

	m_cacheBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	m_cacheBuildMemUsage = m_talloc->high;


	const dtNavMesh* nav = m_navMesh;
	int navmeshMemUsage = 0;
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nav->getTile(i);
		if (tile->header)
			navmeshMemUsage += tile->dataSize;
	}
	printf("navmeshMemUsage = %.1f kB", navmeshMemUsage / 1024.0f);
	
	return 0;
}

int RaycastBuildTempObstacle::saveAll(const char* path)
{
	if (!m_tileCache)
		return 1;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return 2;

	int mversion = FREE_MAPDATA_VERSION;
	fwrite(&mversion, sizeof(int), 1, fp);
	m_tmproc->save(fp);
	// Store header.
	TileCacheSetHeader header;
	header.magic = TILECACHESET_MAGIC;
	header.version = TILECACHESET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
	memcpy(&header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
	rcVcopy(m_BMax, m_geom->getNavMeshBoundsMax());
	header.meshParams.heightMax = m_BMax[1];
	fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < m_tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = m_tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		TileCacheTileHeader tileHeader;
		tileHeader.tileRef = m_tileCache->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);

	return 0;
}

dtNavMesh* RaycastBuildTempObstacle::loadAll(const char* buf, int bufSize)
{
	int mversion;
	bool ret = readBuffer(&buf, bufSize, &mversion, sizeof(int));
	if ( mversion != FREE_MAPDATA_VERSION )
	{
		return 0;
	}
	//m_tmproc->save(fp);

	TileCacheSetHeader header;
	ret = readBuffer(&buf, bufSize, &header, sizeof(TileCacheSetHeader));

	if (!ret)
	{
		return 0;
	}
	if (header.magic != TILECACHESET_MAGIC)
	{
		return 0;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		return 0;
	}

	m_tileCache = dtAllocTileCache();
	if (!m_tileCache)
	{
		return 0;
	}
	dtStatus status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}
	status = mesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	dtVcopy(m_BMin, header.meshParams.orig);
	m_tileWidth = header.meshParams.tileWidth;
	m_tileHeight = header.meshParams.tileHeight;
	m_BMax[0] = m_BMin[0] + header.meshParams.widthTile * header.meshParams.tileWidth;
	m_BMax[1] = header.meshParams.heightMax;
	m_BMax[2] = m_BMin[2] + header.meshParams.heightTile * header.meshParams.tileHeight;

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		TileCacheTileHeader tileHeader;
		ret = readBuffer(&buf, bufSize, &tileHeader, sizeof(tileHeader));
		if (!ret)
		{
			// Error or early EOF
			return 0;
		}
		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		ret = readBuffer(&buf, bufSize, data, tileHeader.dataSize);
		if (!ret)
		{
			// Error or early EOF
			dtFree(data);
			return 0;
		}

		dtCompressedTileRef tile = 0;
		dtStatus addTileStatus = m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
			m_tileCache->buildNavMeshTile(tile, mesh);
	}

	return mesh;
}

int RaycastBuildTempObstacle::addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret)
{
	dtStatus status = m_tileCache->addObstacle(c, r, h, &ret);

	if (dtStatusFailed(status))
	{
		ret = 0;
		return status;
	}
	return 0;
}

int RaycastBuildTempObstacle::addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret)
{
	dtStatus status = m_tileCache->addBoxObstacle(c, ext, rad, &ret);

	if (dtStatusFailed(status))
	{
		ret = 0;
		return status;
	}
	return 0;
}

int RaycastBuildTempObstacle::removeObstacle(unsigned int ref)
{
	dtStatus status = m_tileCache->removeObstacle(ref);

	if (dtStatusFailed(status))
	{
		return status;
	}
	return 0;
}

dtStatus RaycastBuildTempObstacle::applyObstacle(int numNav, class dtNavMesh** navmeshes, int maxTile)
{
	bool upToDate = false;
	while (!upToDate)
	{
		for (int j = 0; j < numNav; j++)
		{
			m_tileCache->update(0, navmeshes[j], &upToDate);
		}
	}
	return 0;
}

int RaycastBuildTempObstacle::assureInMap(float& x, float& y, float& z)
{
	if (x < m_BMin[0])
		x = m_BMin[0];
	if (x > m_BMax[0])
		x = m_BMax[0];
	if (y < m_BMin[1])
		y = m_BMin[1];
	if (y > m_BMax[1])
		y = m_BMax[1];
	if (z < m_BMin[2])
		z = m_BMin[2];
	if (z > m_BMax[2])
		z = m_BMax[2];

	return 0;
}

int RaycastBuildTempObstacle::rasterizeTileLayers(bool genDrop,
	const int tx, const int ty,
	const rcConfig& cfg,
	TileCacheData* tiles,
	const int maxTiles)
{
	if (!m_geom || !m_geom->meshInited())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}

	FastLZCompressor comp;
	RasterizationContext rc;

	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;

	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg.bmin[0] + tx*tcs;
	tcfg.bmin[1] = cfg.bmin[1];
	tcfg.bmin[2] = cfg.bmin[2] + ty*tcs;
	tcfg.bmax[0] = cfg.bmin[0] + (tx + 1)*tcs;
	tcfg.bmax[1] = cfg.bmax[1];
	tcfg.bmax[2] = cfg.bmin[2] + (ty + 1)*tcs;
	tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;

	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	int maxTrisPerChunk = 0;
	std::vector<MeshObj>& meshes = m_geom->getMeshObjs();
	for (int i = 0; i < meshes.size(); i++)
	{
		if (maxTrisPerChunk < meshes[i].chunkyMesh->maxTrisPerChunk)
		{
			maxTrisPerChunk = meshes[i].chunkyMesh->maxTrisPerChunk;
		}
	}
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[maxTrisPerChunk];
	if (!rc.triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", maxTrisPerChunk);
		return 0;
	}

	for (int i = 0; i < meshes.size(); i++)
	{
		const float* verts = meshes[i].mesh->getVerts();
		const int nverts = meshes[i].mesh->getVertCount();
		if (!dtOverlapBounds(tcfg.bmin, tcfg.bmax, meshes[i].meshBMin, meshes[i].meshBMax))
		{
			continue;
		}
		const rcChunkyTriMesh* chunkyMesh = meshes[i].chunkyMesh;
		float tbmin[2], tbmax[2];
		tbmin[0] = tcfg.bmin[0];
		tbmin[1] = tcfg.bmin[2];
		tbmax[0] = tcfg.bmax[0];
		tbmax[1] = tcfg.bmax[2];
		int cid[512];// TODO: Make grow when returning too many items.
		const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
		if (!ncid)
		{
			continue; // empty
		}

		for (int i = 0; i < ncid; ++i)
		{
			const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
			const int* tris = &chunkyMesh->tris[node.i * 3];
			const int ntris = node.n;

			memset(rc.triareas, 0, ntris * sizeof(unsigned char));
			rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
				verts, nverts, tris, ntris, rc.triareas, meshes[i].areaType);

			if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb))
				return 0;
		}
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);


	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts,
			vols[i].hmin, vols[i].hmax,
			(unsigned char)vols[i].area, *rc.chf);
	}

	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}

	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];

		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
			&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}

	return n;
}
