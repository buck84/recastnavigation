#include "recast_build_tiled_dm_dynamic.h"
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

RaycastBuildTiledDropMeshDynamic::RaycastBuildTiledDropMeshDynamic() :
	m_keepInterResults(false),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileBuildTime(0),
	m_tileMemUsage(0),
	m_tileTriCount(0),
	m_tileMeshInfos(0),
	m_buildMode(BM_BUILD),
	m_obstacles(0),
	m_nextFreeObstacle(0),
	m_obstaclesInUse(0),
	m_maxObstacles(10240),
	m_nreqs(0),
	m_nupdate(0)

{
	resetCommonSettings();
}

RaycastBuildTiledDropMeshDynamic::~RaycastBuildTiledDropMeshDynamic()
{
	cleanup();
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	delete[] m_tileMeshInfos;
	m_tileMeshInfos = 0;

	if (m_solidCache)
	{
		for (int i = 0; i < m_heightTile; i++)
		{
			delete[] m_solidCache[i];
		}
		delete[] m_solidCache;
		m_solidCache = 0;
	}
}

void RaycastBuildTiledDropMeshDynamic::cleanup()
{
	delete[] m_triareas;
	m_triareas = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
}

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	int widthTile;
	int heightTile;
	float heightMax;
	float cellSize;
	float cellHeight;
	float maxSlope;
	float dropHeight;
	float dropHoriDistScale;
	float tileSize;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
	int dataSizeDropMesh;
};

inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

int RaycastBuildTiledDropMeshDynamic::addMeshEnd()
{
	int gw = 0, gh = 0;
	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Max tiles and max polys affect how the tile IDs are caculated.
	// There are 22 bits available for identifying a tile and a polygon.
	int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
	if (tileBits > 14) tileBits = 14;
	int polyBits = 22 - tileBits;
	m_maxTiles = 1 << tileBits;
	m_maxPolysPerTile = 1 << polyBits;

	if (m_obstacles)
	{
		delete[] m_obstacles;
	}
	// Alloc space for obstacles.
	m_obstacles = (frObstacle*)dtAlloc(sizeof(frObstacle)*m_maxObstacles, DT_ALLOC_PERM);
	if (!m_obstacles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_obstacles, 0, sizeof(frObstacle)*m_maxObstacles);
	m_nextFreeObstacle = 0;
	m_obstaclesInUse = 0;
	for (int i = m_maxObstacles - 1; i >= 0; --i)
	{
		m_obstacles[i].salt = 1;
		m_obstacles[i].next = m_nextFreeObstacle;
		m_nextFreeObstacle = &m_obstacles[i];
	}

	if (m_tileMeshInfos)
	{
		delete[] m_tileMeshInfos;
		m_tileMeshInfos = 0;
	}
	if (m_widthTile>0)
	{
		m_tileMeshInfos = new tileMeshInfo[m_widthTile*m_heightTile];
	}
	return 0;
}

#ifdef UNITY_EDITOR
extern void NavMeshBuildStep(const char* log, int total, int step);
#endif

int RaycastBuildTiledDropMeshDynamic::build(bool genDrop)
{
	m_buildMode = BM_BUILD;
	if (!m_geom)// || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return 1;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return 2;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	m_jumpDistHori = m_agentRadius * m_dropHoriDistScale + 2 * m_cellSize;
	params.jumpDistHori = m_jumpDistHori;
	params.agentRadius = m_agentRadius;

	dtStatus status;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return 3;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return 4;
	}
	buildAllTiles(m_navMesh, genDrop);

	return 0;
}

int RaycastBuildTiledDropMeshDynamic::buildAllInGame(dtNavMesh*& navmesh)
{
	m_buildMode = BM_LOAD;
	if (!m_geom)// || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return 1;
	}

	dtFreeNavMesh(navmesh);

	navmesh = dtAllocNavMesh();
	if (!navmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return 2;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin());
	dtVcopy(m_BMin, params.orig);
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	m_jumpDistHori = m_agentRadius * m_dropHoriDistScale + 2 * m_cellSize;
	params.jumpDistHori = m_jumpDistHori;
	params.agentRadius = m_agentRadius;

	dtStatus status;

	status = navmesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return 3;
	}

	status = m_navQuery->init(navmesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return 4;
	}
	buildAllTiles(navmesh, true);

	return 0;
}

void RaycastBuildTiledDropMeshDynamic::buildAllTiles(class dtNavMesh* navmesh, bool genDrop)
{
	if (!m_geom) return;
	if (!navmesh) return;

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;
	const float tcs = m_tileSize*m_cellSize;
	
	if (m_solidCache)
	{
		for (int i = 0; i < m_heightTile; i++)
		{
			delete[] m_solidCache[i];
		}
		delete[] m_solidCache;
		m_solidCache = 0;
	}
	m_widthTile = tw;
	m_heightTile = th;
	m_tileWidth = tcs;
	m_tileHeight = tcs;
	m_solidCache = new rcHeightfield*[th];
	m_BMax[0] = m_BMin[0] + m_widthTile * m_tileWidth;
	m_BMax[1] = bmax[1];
	m_BMax[2] = m_BMin[2] + m_heightTile * m_tileHeight;

	if (!m_tileMeshInfos)
	{
		m_tileMeshInfos = new tileMeshInfo[m_widthTile*m_heightTile];
	}

	for (int i = 0; i < m_heightTile; i++)
	{
		m_solidCache[i] = new rcHeightfield[tw];
		memset(m_solidCache[i], 0, tw * sizeof(rcHeightfield));
	}

	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
#ifdef UNITY_EDITOR
			NavMeshBuildStep("rasterizeTileLayers", th*tw, y*tw + x);
#endif
			float m_lastBuiltTileBmin[3];
			float m_lastBuiltTileBmax[3];
			m_lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			m_lastBuiltTileBmin[1] = bmin[1];
			m_lastBuiltTileBmin[2] = bmin[2] + y*tcs;

			m_lastBuiltTileBmax[0] = bmin[0] + (x + 1)*tcs;
			m_lastBuiltTileBmax[1] = bmax[1];
			m_lastBuiltTileBmax[2] = bmin[2] + (y + 1)*tcs;

			int dataSize = 0;
			unsigned char* data = buildTileMesh(x, y, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize);
			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				navmesh->removeTile(navmesh->getTileRefAt(x, y, 0), 0, 0);
				// Let the navmesh own the data.
				dtStatus status = navmesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);
				if (dtStatusFailed(status))
					dtFree(data);
			}
		}
	}

	for (int i = 0; i < navmesh->getMaxTiles(); ++i)
	{
#ifdef UNITY_EDITOR
		NavMeshBuildStep("buildTileDropMesh", navmesh->getMaxTiles(), i);
#endif
		int dataSize = 0;
		unsigned char* data = buildTileDropMesh(i, dataSize, navmesh);
		if (data)
		{
			navmesh->loadTileDropMesh(i, data, dataSize);
		}
		else
		{

		}
	}
	/*
	if (m_solidCache)
	{
		for (int i = 0; i < m_heightTile; i++)
		{
			delete[] m_solidCache[i];
		}
		delete[] m_solidCache;
		m_solidCache = 0;
	}
	*/
	m_solid = 0;

	// Start the build process.	
	m_ctx->stopTimer(RC_TIMER_TEMP);
}

int RaycastBuildTiledDropMeshDynamic::saveAll(const char* path)
{
	const dtNavMesh* mesh = m_navMesh;
	if (!mesh)
		return 11;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return 12;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	header.widthTile = m_widthTile;
	header.heightTile = m_heightTile;
	header.cellSize = m_cellSize;
	header.cellHeight = m_cellHeight;
	header.maxSlope = m_agentMaxSlope;
	header.dropHeight = m_dropHeight;
	header.dropHoriDistScale = m_dropHoriDistScale;
	header.tileSize = m_tileSize;

	rcVcopy(m_BMax, m_geom->getNavMeshBoundsMax());
	header.heightMax = m_BMax[1];
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		tileHeader.dataSizeDropMesh = tile->dataSizeDropMesh;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
		if (tile->dataSizeDropMesh > 0)
		{
			fwrite(tile->dataDropMesh, tile->dataSizeDropMesh, 1, fp);
		}
	}

	fclose(fp);
	return 0;
}

dtNavMesh* RaycastBuildTiledDropMeshDynamic::loadAll(const char* buf, int bufSize)
{
	m_buildMode = BM_LOAD;
	// Read header.
	NavMeshSetHeader header;
	readBuffer(&buf, bufSize, &header, sizeof(NavMeshSetHeader));
	if (header.magic != NAVMESHSET_MAGIC)
	{
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		return 0;
	}

	m_widthTile = header.widthTile;
	m_heightTile = header.heightTile;
	dtVcopy(m_BMin, header.params.orig);
	m_tileWidth = header.params.tileWidth;
	m_tileHeight = header.params.tileHeight;
	m_BMax[0] = m_BMin[0] + header.widthTile * header.params.tileWidth;
	m_BMax[1] = header.heightMax;
	m_BMax[2] = m_BMin[2] + header.heightTile * header.params.tileHeight;
	
	m_cellSize = header.cellSize;
	m_cellHeight = header.cellHeight;
	m_agentMaxSlope = header.maxSlope;
	m_dropHeight = header.dropHeight;
	m_dropHoriDistScale = header.dropHoriDistScale;
	m_tileSize = header.tileSize;

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		if (!readBuffer(&buf, bufSize, &tileHeader, sizeof(NavMeshTileHeader)))
		{
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		if (!readBuffer(&buf, bufSize, data, tileHeader.dataSize))
		{
			return 0;
		}

		unsigned char* dataDrop = 0;
		if (tileHeader.dataSizeDropMesh > 0)
		{
			dataDrop = (unsigned char*)dtAlloc(tileHeader.dataSizeDropMesh, DT_ALLOC_PERM);
			if (!dataDrop) break;
			memset(dataDrop, 0, tileHeader.dataSizeDropMesh);
			if (!readBuffer(&buf, bufSize, dataDrop, tileHeader.dataSizeDropMesh))
			{
				return 0;
			}
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0, dataDrop);
	}

	if (m_solidCache)
	{
		for (int i = 0; i < m_heightTile; i++)
		{
			delete[] m_solidCache[i];
		}
		delete[] m_solidCache;
		m_solidCache = 0;
	}
	m_solidCache = new rcHeightfield*[m_heightTile];
	for (int i = 0; i < m_heightTile; i++)
	{
		m_solidCache[i] = new rcHeightfield[m_widthTile];
		memset(m_solidCache[i], 0, m_widthTile*sizeof(rcHeightfield));
	}

	return mesh;
}

dtNavMesh* RaycastBuildTiledDropMeshDynamic::buildAll()
{
	/*
	m_buildMode = BM_LOAD;
	// Read header.
	NavMeshSetHeader header;
	readBuffer(&buf, bufSize, &header, sizeof(NavMeshSetHeader));
	if (header.magic != NAVMESHSET_MAGIC)
	{
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		return 0;
	}

	m_widthTile = header.widthTile;
	m_heightTile = header.heightTile;
	dtVcopy(m_BMin, header.params.orig);
	m_tileWidth = header.params.tileWidth;
	m_tileHeight = header.params.tileHeight;
	m_BMax[0] = m_BMin[0] + header.widthTile * header.params.tileWidth;
	m_BMax[1] = header.heightMax;
	m_BMax[2] = m_BMin[2] + header.heightTile * header.params.tileHeight;

	m_cellSize = header.cellSize;
	m_cellHeight = header.cellHeight;
	m_agentMaxSlope = header.maxSlope;
	m_dropHeight = header.dropHeight;
	m_dropHoriDistScale = header.dropHoriDistScale;
	m_tileSize = header.tileSize;

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		if (!readBuffer(&buf, bufSize, &tileHeader, sizeof(NavMeshTileHeader)))
		{
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		if (!readBuffer(&buf, bufSize, data, tileHeader.dataSize))
		{
			return 0;
		}

		unsigned char* dataDrop = 0;
		if (tileHeader.dataSizeDropMesh > 0)
		{
			dataDrop = (unsigned char*)dtAlloc(tileHeader.dataSizeDropMesh, DT_ALLOC_PERM);
			if (!dataDrop) break;
			memset(dataDrop, 0, tileHeader.dataSizeDropMesh);
			if (!readBuffer(&buf, bufSize, dataDrop, tileHeader.dataSizeDropMesh))
			{
				return 0;
			}
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0, dataDrop);
	}

	if (m_solidCache)
	{
		for (int i = 0; i < m_heightTile; i++)
		{
			delete[] m_solidCache[i];
		}
		delete[] m_solidCache;
		m_solidCache = 0;
	}
	m_solidCache = new rcHeightfield*[m_heightTile];
	for (int i = 0; i < m_heightTile; i++)
	{
		m_solidCache[i] = new rcHeightfield[m_widthTile];
		memset(m_solidCache[i], 0, m_widthTile * sizeof(rcHeightfield));
	}

	return mesh;
	*/
	return 0;
}

int RaycastBuildTiledDropMeshDynamic::addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret)
{
	frObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = m_obstaclesInUse;
		m_obstaclesInUse = ob;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	ret = getObstacleRef(ob);
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_CYLINDER;
	ob->ntouched = 0;
	dtVcopy(ob->cylinder.pos, c);
	ob->cylinder.radius = r;
	ob->cylinder.height = h;
	if (ob->mesh)
		delete ob->mesh;
	ob->mesh = new rcMeshLoaderObj();

	float vertices[88 * 3] = {
		-0.476f, -1.000f, -0.155f,
		-0.405f, -1.000f, -0.294f,
		-0.294f, -1.000f, -0.405f,
		-0.155f, -1.000f, -0.476f,
		0.000f, -1.000f, -0.500f,
		0.155f, -1.000f, -0.476f,
		0.294f, -1.000f, -0.405f,
		0.405f, -1.000f, -0.294f,
		0.476f, -1.000f, -0.155f,
		0.500f, -1.000f, 0.000f,
		0.476f, -1.000f, 0.155f,
		0.405f, -1.000f, 0.294f,
		0.294f, -1.000f, 0.405f,
		0.155f, -1.000f, 0.476f,
		0.000f, -1.000f, 0.500f,
		-0.155f, -1.000f, 0.476f,
		-0.294f, -1.000f, 0.405f,
		-0.405f, -1.000f, 0.294f,
		-0.476f, -1.000f, 0.155f,
		-0.500f, -1.000f, 0.000f,
		-0.476f, 1.000f, -0.155f,
		-0.405f, 1.000f, -0.294f,
		-0.294f, 1.000f, -0.405f,
		-0.155f, 1.000f, -0.476f,
		0.000f, 1.000f, -0.500f,
		0.155f, 1.000f, -0.476f,
		0.294f, 1.000f, -0.405f,
		0.405f, 1.000f, -0.294f,
		0.476f, 1.000f, -0.155f,
		0.500f, 1.000f, 0.000f,
		0.476f, 1.000f, 0.155f,
		0.405f, 1.000f, 0.294f,
		0.294f, 1.000f, 0.405f,
		0.155f, 1.000f, 0.476f,
		0.000f, 1.000f, 0.500f,
		-0.155f, 1.000f, 0.476f,
		-0.294f, 1.000f, 0.405f,
		-0.405f, 1.000f, 0.294f,
		-0.476f, 1.000f, 0.155f,
		-0.500f, 1.000f, 0.000f,
		0.000f, -1.000f, 0.000f,
		0.000f, 1.000f, 0.000f,
		0.500f, -1.000f, 0.000f,
		0.500f, 1.000f, 0.000f,
		-0.500f, -1.000f, 0.000f,
		-0.500f, 1.000f, 0.000f,
		-0.476f, 1.000f, -0.155f,
		-0.476f, -1.000f, -0.155f,
		-0.405f, -1.000f, -0.294f,
		-0.476f, -1.000f, -0.155f,
		-0.294f, -1.000f, -0.405f,
		-0.155f, -1.000f, -0.476f,
		0.000f, -1.000f, -0.500f,
		0.155f, -1.000f, -0.476f,
		0.294f, -1.000f, -0.405f,
		0.405f, -1.000f, -0.294f,
		0.476f, -1.000f, -0.155f,
		0.500f, -1.000f, 0.000f,
		0.476f, -1.000f, 0.155f,
		0.405f, -1.000f, 0.294f,
		0.294f, -1.000f, 0.405f,
		0.155f, -1.000f, 0.476f,
		0.000f, -1.000f, 0.500f,
		-0.155f, -1.000f, 0.476f,
		-0.294f, -1.000f, 0.405f,
		-0.405f, -1.000f, 0.294f,
		-0.476f, -1.000f, 0.155f,
		-0.500f, -1.000f, 0.000f,
		-0.476f, 1.000f, -0.155f,
		-0.405f, 1.000f, -0.294f,
		-0.294f, 1.000f, -0.405f,
		-0.155f, 1.000f, -0.476f,
		0.000f, 1.000f, -0.500f,
		0.155f, 1.000f, -0.476f,
		0.294f, 1.000f, -0.405f,
		0.405f, 1.000f, -0.294f,
		0.476f, 1.000f, -0.155f,
		0.500f, 1.000f, 0.000f,
		0.476f, 1.000f, 0.155f,
		0.405f, 1.000f, 0.294f,
		0.294f, 1.000f, 0.405f,
		0.155f, 1.000f, 0.476f,
		0.000f, 1.000f, 0.500f,
		-0.155f, 1.000f, 0.476f,
		-0.294f, 1.000f, 0.405f,
		-0.405f, 1.000f, 0.294f,
		-0.476f, 1.000f, 0.155f,
		-0.500f, 1.000f, 0.000f,
	};
	int vCount = 88;
	for (int i = 0; i < vCount; i++)
	{
		vertices[i * 3] = c[0] + r * vertices[i * 3] * 2;
		vertices[i * 3 + 1] = c[1] + h * vertices[i * 3 + 1] * 2;
		vertices[i * 3 + 2] = c[2] + r * vertices[i * 3 + 2] * 2;
	}

	int triangles[80*3] = {
		0, 20, 21,
		0, 21, 1,
		1, 21, 22,
		1, 22, 2,
		2, 22, 23,
		2, 23, 3,
		3, 23, 24,
		3, 24, 4,
		4, 24, 25,
		4, 25, 5,
		5, 25, 26,
		5, 26, 6,
		6, 26, 27,
		6, 27, 7,
		7, 27, 28,
		7, 28, 8,
		8, 28, 29,
		8, 29, 9,
		42, 43, 30,
		42, 30, 10,
		10, 30, 31,
		10, 31, 11,
		11, 31, 32,
		11, 32, 12,
		12, 32, 33,
		12, 33, 13,
		13, 33, 34,
		13, 34, 14,
		14, 34, 35,
		14, 35, 15,
		15, 35, 36,
		15, 36, 16,
		16, 36, 37,
		16, 37, 17,
		17, 37, 38,
		17, 38, 18,
		18, 38, 39,
		18, 39, 19,
		44, 45, 46,
		44, 46, 47,
		48, 40, 49,
		50, 40, 48,
		51, 40, 50,
		52, 40, 51,
		53, 40, 52,
		54, 40, 53,
		55, 40, 54,
		56, 40, 55,
		57, 40, 56,
		58, 40, 57,
		59, 40, 58,
		60, 40, 59,
		61, 40, 60,
		62, 40, 61,
		63, 40, 62,
		64, 40, 63,
		65, 40, 64,
		66, 40, 65,
		67, 40, 66,
		49, 40, 67,
		68, 41, 69,
		69, 41, 70,
		70, 41, 71,
		71, 41, 72,
		72, 41, 73,
		73, 41, 74,
		74, 41, 75,
		75, 41, 76,
		76, 41, 77,
		77, 41, 78,
		78, 41, 79,
		79, 41, 80,
		80, 41, 81,
		81, 41, 82,
		82, 41, 83,
		83, 41, 84,
		84, 41, 85,
		85, 41, 86,
		86, 41, 87,
		87, 41, 68,
	};
	int tCount = 80;
	ob->mesh->init(vertices, vCount, triangles, tCount);

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	return 0;
}

int RaycastBuildTiledDropMeshDynamic::addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret)
{
	frObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = m_obstaclesInUse;
		m_obstaclesInUse = ob;
	}
	if (!ob)
		return 1;

	ret = getObstacleRef(ob);
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_ORIENTED_BOX;
	ob->ntouched = 0;
	dtVcopy(ob->orientedBox.center, c);
	dtVscale(ob->orientedBox.halfExtents, ext, 0.5f);

	float coshalf = cosf(0.5f*rad);
	float sinhalf = sinf(-0.5f*rad);
	ob->orientedBox.rotAux[0] = coshalf*sinhalf;
	ob->orientedBox.rotAux[1] = coshalf*coshalf - 0.5f;

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);

	float vertices[24 * 3] = {
		0.5f, -0.5f, 0.5f,
		-0.5f, -0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, -0.5f,
		-0.5f, 0.5f, -0.5f,
		0.5f, -0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,
		0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, -0.5f,
		-0.5f, 0.5f, -0.5f,
		0.5f, -0.5f, -0.5f,
		0.5f, -0.5f, 0.5f,
		-0.5f, -0.5f, 0.5f,
		-0.5f, -0.5f, -0.5f,
		-0.5f, -0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, -0.5f,
		-0.5f, -0.5f, -0.5f,
		0.5f, -0.5f, -0.5f,
		0.5f, 0.5f, -0.5f,
		0.5f, 0.5f, 0.5f,
		0.5f, -0.5f, 0.5f,
	};
	int vCount = 24;
	float cosRad = cosf(rad);
	float sinRad = sinf(rad);
	for (int i = 0; i < vCount; i++)
	{
		float x = ext[0] * vertices[i * 3];
		float z = ext[2] * vertices[i * 3 + 2];
		vertices[i * 3] = c[0] + x*cosRad - z*sinRad;
		vertices[i * 3 + 1] = c[1] + ext[1] * vertices[i * 3 + 1];
		vertices[i * 3 + 2] = c[2] + x*sinRad + z*cosRad;

	}

	int triangles[36] = {
		0, 2, 3,
		0, 3, 1,
		8, 4, 5,
		8, 5, 9,
		10, 6, 7,
		10, 7, 11,
		12, 13, 14,
		12, 14, 15,
		16, 17, 18,
		16, 18, 19,
		20, 21, 22,
		20, 22, 23,
	};
	int tCount = 12;
	
	if (ob->mesh)
		delete ob->mesh;
	ob->mesh = new rcMeshLoaderObj();
	ob->mesh->init(vertices, vCount, triangles, tCount);
	return 0;// ob - m_obstacles;
}

int RaycastBuildTiledDropMeshDynamic::removeObstacle(unsigned int ref)
{
	if (!ref)
		return 1;

	frObstacle* testObstacle = m_obstaclesInUse;
	frObstacle* testObstaclePrev = 0;
	while (testObstacle)
	{
		int refTest = getObstacleRef(testObstacle);
		if (refTest == ref)
		{
			if (testObstaclePrev)
			{
				testObstaclePrev->next = testObstacle->next;
			}
			else
				m_obstaclesInUse = testObstacle->next;
			break;
		}
		testObstaclePrev = testObstacle;
		testObstacle = testObstacle->next;
	}

	if (testObstacle == 0)
		return DT_FAILURE;

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_REMOVE;
	req->ref = ref;
	return 0;
}

dtStatus RaycastBuildTiledDropMeshDynamic::buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh)
{
	float bmin[3], bmax[3];
	const float tcs = m_tileWidth;
	bmin[0] = m_BMin[0] + tx*tcs;
	bmin[1] = m_BMin[1];
	bmin[2] = m_BMin[2] + ty*tcs;
	bmax[0] = m_BMin[0] + (tx+1)*tcs;
	bmax[1] = m_BMax[1];
	bmax[2] = m_BMin[2] + (ty + 1)*tcs;
	int dataSize = 0;
	unsigned char* data = buildTileMesh(tx, ty, bmin, bmax, dataSize);
	if (data)
	{
		// Remove any previous data (navmesh owns and deletes the data).
		navmesh->removeTile(navmesh->getTileRefAt(tx, ty, 0), 0, 0);
		// Let the navmesh own the data.
		dtStatus status = navmesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
			dtFree(data);
		return DT_SUCCESS;
	}


	return DT_FAILURE;
}

unsigned char* RaycastBuildTiledDropMeshDynamic::buildTileMesh(const int tx, const int ty, const float* bmin, const float* bmax, int& dataSize)
{
	tileMeshInfo* tmi;
	tileMeshInfo tmiCache;
	if (m_buildMode== BM_BUILD)
	{
		tmi = &tmiCache;
	}
	else
	{
		tmi = &m_tileMeshInfos[ty*m_widthTile + tx];
	}
	/*
	if (!m_geom || !m_geom->getMesh() || !m_geom->getChunkyMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return 0;
	}
	*/

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	cleanup();

	// Init build configuration from GUI
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.tileSize = (int)m_tileSize;
	m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding.
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// No polygons (or contours) will be created on the border area.
	//
	// IMPORTANT!
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build
	//   : |     | :  
	//   : +-----+ :<-- geometry needed
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	m_cfg.bmin[0] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmin[2] -= m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[0] += m_cfg.borderSize*m_cfg.cs;
	m_cfg.bmax[2] += m_cfg.borderSize*m_cfg.cs;

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);

	// Allocate voxel heightfield where we rasterize our input data to.
	m_solid = &m_solidCache[ty][tx];

	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	m_solid->release();
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	std::vector<MeshObj>& meshes = m_geom->getMeshObjs();
	if (!tmi->inited)
	{
		for (int i = 0; i < meshes.size(); i++)
		{
			if (!dtOverlapBounds(m_cfg.bmin, m_cfg.bmax, meshes[i].meshBMin, meshes[i].meshBMax))
			{
				continue;
			}
			const rcChunkyTriMesh* chunkyMesh = meshes[i].chunkyMesh;
			float tbmin[2], tbmax[2];
			tbmin[0] = m_cfg.bmin[0];
			tbmin[1] = m_cfg.bmin[2];
			tbmax[0] = m_cfg.bmax[0];
			tbmax[1] = m_cfg.bmax[2];
			int cid[512];// TODO: Make grow when returning too many items.
			const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
			if (!ncid)
			{
				continue; // empty
			}
			MeshInfo mi;
			mi.index = i;
			for (int i1=0; i1<ncid; i1++)
			{
				mi.cids.push_back(cid[i1]);
			}

			tmi->meshes.push_back(mi);
		}
		tmi->inited = true;
	}
	int maxTrisPerChunk = 0;
	for (int i = 0; i < meshes.size(); i++)
	{
		if (maxTrisPerChunk < meshes[i].chunkyMesh->maxTrisPerChunk)
		{
			maxTrisPerChunk = meshes[i].chunkyMesh->maxTrisPerChunk;
		}
	}
	frObstacle* ob = m_obstaclesInUse;
	while (ob)
	{
		if (ob->state == DT_OBSTACLE_EMPTY)
		{
			ob = ob->next;
			continue;
		}
		const int ntris = ob->mesh->getTriCount();
		if (maxTrisPerChunk < ntris)
		{
			maxTrisPerChunk = ntris;
		}
		ob = ob->next;
	}
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	m_triareas = new unsigned char[maxTrisPerChunk];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", maxTrisPerChunk);
		return 0;
	}

	for (int i1=0; i1<tmi->meshes.size(); i1++)
	{
		MeshInfo& mInfo = tmi->meshes[i1];
		const float* verts = meshes[mInfo.index].mesh->getVerts();
		const int nverts = meshes[mInfo.index].mesh->getVertCount();
		const int ntris = meshes[mInfo.index].mesh->getTriCount();
		const rcChunkyTriMesh* chunkyMesh = meshes[mInfo.index].chunkyMesh;
		const int ncid = mInfo.cids.size();
		if (!ncid)
			return 0;

		m_tileTriCount = 0;

		for (int i2 = 0; i2 < ncid; ++i2)
		{
			const rcChunkyTriMeshNode& node = chunkyMesh->nodes[mInfo.cids[i2]];
			const int* ctris = &chunkyMesh->tris[node.i * 3];
			const int nctris = node.n;

			m_tileTriCount += nctris;

			memset(m_triareas, 0, nctris * sizeof(unsigned char));
			rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
				verts, nverts, ctris, nctris, m_triareas, meshes[i1].areaType);
			if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas, nctris, *m_solid, m_cfg.walkableClimb))
				return 0;
		}
	}

	ob = m_obstaclesInUse;
	while (ob)
	{
		if (ob->state == DT_OBSTACLE_EMPTY)
		{
			ob = ob->next;
			continue;
		}
		const float* verts = ob->mesh->getVerts();
		const int nverts = ob->mesh->getVertCount();
		const int* tris = ob->mesh->getTris();
		const int ntris = ob->mesh->getTriCount();

		memset(m_triareas, 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
			verts, nverts, tris, ntris, m_triareas, 1);

		if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
			return 0;

		ob = ob->next;
	}

	if (!m_keepInterResults)
	{
		delete[] m_triareas;
		m_triareas = 0;
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);


	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heighfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return 0;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return 0;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return 0;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return 0;
		}
	}

	// Create contours.
	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return 0;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return 0;
	}

	if (m_cset->nconts == 0)
	{
		return 0;
	}

	// Build polygon navmesh from the contours.
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return 0;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return 0;
	}

	// Build detail mesh.
	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
		return 0;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf,
		m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
		*m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
		return 0;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			m_ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_pmesh->nverts, 0xffff);
			return 0;
		}

		// Update poly flags from areas.
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
	}
	m_tileMemUsage = navDataSize / 1024.0f;

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

	m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

	dataSize = navDataSize;
	return navData;
}

struct dropBuildInfo
{
	dtPolyRef ref;
	float lmin;	// real length
	float lmax; // real length
	float hmin;
	float hmax;
};

extern void addDropData(int &index, dtDropData* dropData, int tileId, int polyId, int edgeId, dtPolyRef ref,
	unsigned char bmin, unsigned char bmax, unsigned char lowDrop);

unsigned char* RaycastBuildTiledDropMeshDynamic::buildTileDropMesh(int index, int& dataSize, class dtNavMesh* navmesh)
{
	const dtNavMesh& mesh = *navmesh;
	const dtMeshTile* tile = mesh.getTile(index);
	if (!tile->header)
		return 0;

	m_jumpDistHori = navmesh->getJumpDistHori();
	m_navQuery->init(navmesh, 2048);
	dtDropData* dropData = new dtDropData[tile->header->maxLinkCount];
	int numDropData = 0;
	float ptTestObstacle[4][3];
	float pTestv[3];
	float pTestv0[3];
	float pTestv1[3];
	static const int dropEdgeMax = 16;
	dropBuildInfo dropEdges[dropEdgeMax];
	dropBuildInfo drops[dropEdgeMax];

	for (int i1 = 0; i1 < tile->header->polyCount; ++i1)
	{
		if (i1>9)
		{
			int a = 3;
			a = 4;
		}
		const dtPoly* p = &tile->polys[i1];

		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION ||
			p->getType() == DT_POLYTYPE_OFFMESH_DROP)
			continue;
		for (int i2 = 0, nj = (int)p->vertCount; i2 < nj; ++i2)
		{
			if (p->neis[i2] != 0)
			{
				if ((p->neis[i2] & DT_EXT_LINK) == 0)
					continue;
				bool con = false;
				for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
				{
					if (tile->links[k].edge == i2)
					{
						con = true;
						break;
					}
				}
				if (con)
					continue;
			}

			/*

			------deltaEdge------->
			v0  pTestv             v1
			\                       \ nrm
			\   pTestv0             \
			|                       |
			|                       |

			ptTestObstacle[0] ptTestObstacle[1]
			ptTestObstacle[3] ptTestObstacle[2]

			*/

			// v0 v1
			const float* v0 = &tile->verts[p->verts[i2] * 3];
			const float* v1 = &tile->verts[p->verts[(i2 + 1) % nj] * 3];

			// deltaEdge
			float deltaEdgeNorm[3];
			dtVsub(deltaEdgeNorm, v1, v0);
			float heightDeltaNorm = deltaEdgeNorm[1] / dtVlen(deltaEdgeNorm);
			float edgeLen = dtMathSqrtf(dtVlen2DSqr(deltaEdgeNorm));
			dtVscale(deltaEdgeNorm, deltaEdgeNorm, 1 / edgeLen);

			// nrm
			float nrm[3];
			nrm[0] = -deltaEdgeNorm[2];
			nrm[1] = 0;
			nrm[2] = deltaEdgeNorm[0];
			dtVscale(nrm, nrm, m_jumpDistHori);

			// init
			memset(dropEdges, 0, dropEdgeMax * sizeof(dropBuildInfo));
			int numDropEdge = 0;

			// test the whole edge
			dtVcopy(ptTestObstacle[0], v0);
			dtVcopy(ptTestObstacle[1], v1);
			dtVadd(ptTestObstacle[2], v1, nrm);
			dtVadd(ptTestObstacle[3], v0, nrm);
			if (dropNoObstacleHori(ptTestObstacle, navmesh))
			{
				dropEdges[numDropEdge].lmin = 0;
				dropEdges[numDropEdge].lmax = edgeLen;
				numDropEdge = 1;
			}
			else // test every #m_agentRadius length
			{
				float deltaEdgeCollideTest[3];
				dtVscale(deltaEdgeCollideTest, deltaEdgeNorm, m_agentRadius);
				bool setStart = false;
				for (int i = 0; ; i++)
				{
					float testLen = i*m_cellSize;
					if (testLen > edgeLen)
					{
						if (setStart)
						{
							dropEdges[numDropEdge++].lmax = i*m_cellSize;
						}
						break;
					}
					dtVlerp(pTestv, v0, v1, testLen / edgeLen);
					dtVadd(pTestv0, pTestv, nrm);
					dtVadd(ptTestObstacle[0], pTestv, deltaEdgeCollideTest);
					dtVsub(ptTestObstacle[1], pTestv, deltaEdgeCollideTest);
					dtVadd(ptTestObstacle[2], pTestv0, deltaEdgeCollideTest);
					dtVsub(ptTestObstacle[3], pTestv0, deltaEdgeCollideTest);
					if (dropNoObstacleHori(ptTestObstacle, navmesh))
					{
						if (!setStart)
						{
							setStart = true;
							dropEdges[numDropEdge].lmin = i*m_cellSize;
						}
					}
					else
					{
						if (setStart)
						{
							setStart = false;
							dropEdges[numDropEdge++].lmax = (i - 1)*m_cellSize;
						}
					}
				}
			}
			// assert memory overflow
			assert(numDropEdge < dropEdgeMax);
			// init
			memset(drops, 0, dropEdgeMax * sizeof(dropBuildInfo));
			int numDrop = 0;
			// deal every section
			for (int j0 = 0; j0<numDropEdge; j0++)
			{
				float testDist = dropEdges[j0].lmin;
				float dropPoint0[3];
				float dropPoint1[3];
				while (testDist <= dropEdges[j0].lmax)
				{
					// query overlay drop point
					float testDistScale = testDist / edgeLen;
					dtVlerp(pTestv, v0, v1, testDistScale);
					dtVadd(pTestv0, pTestv, nrm);
					dtPolyRef prs[5];
					int prCount = 0;
					float ext[3] = { 0.1f, 15, 0.1f };
					dtQueryFilter filter;
					dtStatus ret = m_navQuery->queryPolygons(pTestv0, ext, &filter, prs, &prCount, 5);
					// find highest drop point
					float posTest[3];
					bool overlay = false;
					float dropHeight = -99999;
					dtPolyRef dropPr = 0;
					for (int i = 0; i < prCount; i++)
					{
						m_navQuery->closestPointOnPoly(prs[i], pTestv0, posTest, &overlay);
						if (overlay)
						{
							if (posTest[1]<(pTestv0[1] - m_agentRadius) && posTest[1]>(pTestv0[1] - m_dropHeight))
							{
								if (dropHeight<posTest[1])
								{
									dropPr = prs[i];
									dropHeight = posTest[1];
								}
							}
						}
					}
					// find drop poly
					if (dropPr != 0)
					{
						drops[numDrop].ref = dropPr;
						drops[numDrop].lmin = testDist;
						drops[numDrop].hmin = pTestv0[1] - dropHeight;
						float deltaEdgeRaycast[3];
						float testEdgeLen = dropEdges[j0].lmax - testDist;
						dtVscale(deltaEdgeRaycast, deltaEdgeNorm, testEdgeLen);
						dtVadd(pTestv1, pTestv0, deltaEdgeRaycast);
						dtVcopy(dropPoint0, pTestv0);
						dropPoint0[1] = dropHeight;
						dtVadd(dropPoint1, dropPoint0, deltaEdgeRaycast);
						float t;
						float hitNormal[3];
						static const int MAX_POLYS = 256;
						dtPolyRef polys[MAX_POLYS];
						float ts[MAX_POLYS];
						int npolys;
						dtStatus ret = m_navQuery->raycast(dropPr, dropPoint0, dropPoint1, &filter, &t, ts, hitNormal, polys, &npolys, MAX_POLYS);
						drops[numDrop].lmax = testDist + testEdgeLen * ts[0];
						float heightTestDrop[3];
						bool overlay;
						float heightTestLand[3];
						dtVlerp(heightTestDrop, pTestv0, pTestv1, ts[0]);
						dtVadd(heightTestDrop, heightTestDrop, nrm);
						m_navQuery->closestPointOnPoly(polys[0], heightTestDrop, heightTestLand, &overlay);
						drops[numDrop].hmax = heightTestDrop[1] - heightTestLand[1];
						numDrop++;
						for (int j1 = 1; j1 < npolys; j1++)
						{
							drops[numDrop].ref = polys[j1];
							drops[numDrop].lmin = drops[numDrop - 1].lmax;
							drops[numDrop].hmin = drops[numDrop - 1].hmax;
							drops[numDrop].lmax = testDist + testEdgeLen * ts[j1];
							dtVlerp(heightTestDrop, pTestv0, pTestv1, ts[j1]);
							dtVadd(heightTestDrop, heightTestDrop, nrm);
							m_navQuery->closestPointOnPoly(polys[j1], heightTestDrop, heightTestLand, &overlay);
							drops[numDrop].hmax = heightTestDrop[1] - heightTestLand[1];
							numDrop++;
						}
						if (t >= 1.0f)
						{
							break;
						}
						else
						{
							testDist += t*testEdgeLen;
						}
					}
					testDist += m_agentRadius;
				}
			}
			assert(numDrop < dropEdgeMax);
			float scale = 255.0f / edgeLen;
			for (int i = 0; i<numDrop; i++)
			{
				if (drops[i].hmin > m_dropHeightAutoMove && drops[i].hmax > m_dropHeightAutoMove)
				{
					addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(drops[i].lmin*scale),
						(unsigned char)(drops[i].lmax*scale), (unsigned char)0);
				}
				else if (drops[i].hmin < m_dropHeightAutoMove && drops[i].hmax < m_dropHeightAutoMove)
				{
					addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(drops[i].lmin*scale),
						(unsigned char)(drops[i].lmax*scale), (unsigned char)1);
				}
				else
				{
					float startPoint = drops[i].lmin;
					float endPoint = drops[i].lmax;
					float t = (drops[i].hmin - m_dropHeightAutoMove) / (drops[i].hmin - drops[i].hmax);
					float midPoint = startPoint + (endPoint - startPoint)*t;
					if (drops[i].hmin > m_dropHeightAutoMove)
					{
						addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(startPoint*scale),
							(unsigned char)(midPoint*scale), (unsigned char)0);
						addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(midPoint*scale),
							(unsigned char)(endPoint*scale), (unsigned char)1);
					}
					else
					{
						addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(startPoint*scale),
							(unsigned char)(midPoint*scale), (unsigned char)1);
						addDropData(numDropData, dropData, index, i1, i2, drops[i].ref, (unsigned char)(midPoint*scale),
							(unsigned char)(endPoint*scale), (unsigned char)0);
					}
				}
			}
		}
	}
	tile->header->dropEdgeCount = numDropData;
	dataSize = numDropData * sizeof(dtDropData);
	if (numDropData > 0)
	{
		return reinterpret_cast<unsigned char*>(dropData);
	}
	else
	{
		delete[]dropData;
		return 0;
	}
}

bool RaycastBuildTiledDropMeshDynamic::dropNoObstacleHori(float p[4][3], class dtNavMesh* navmesh)
{
	float testMin[3], testMax[3];
	testMin[0] = p[0][0];
	testMin[2] = p[0][2];
	testMax[0] = p[0][0];
	testMax[2] = p[0][2];
	testMin[1] = p[0][1];
	for (int i = 1; i < 4; i++)
	{
		testMin[0] = p[i][0] < testMin[0] ? p[i][0] : testMin[0];
		testMin[2] = p[i][2] < testMin[2] ? p[i][2] : testMin[2];
		testMax[0] = p[i][0] > testMax[0] ? p[i][0] : testMax[0];
		testMax[2] = p[i][2] > testMax[2] ? p[i][2] : testMax[2];

		testMin[1] = p[i][1] > testMin[1] ? p[i][1] : testMin[1];
	}

	testMin[1] = testMin[1] + m_cellHeight * 3;
	testMax[1] = testMin[1] + m_cellHeight;

	int minx, miny, maxx, maxy;
	navmesh->calcTileLoc(testMin, &minx, &miny);
	navmesh->calcTileLoc(testMax, &maxx, &maxy);
	for (int y = miny; y <= maxy; y++)
	{
		for (int x = minx; x <= maxx; x++)
		{
			if (x < 0 || x >= m_widthTile || y < 0 || y >= m_heightTile)
				continue;
			if (insectSolid(testMin, testMax, x, y))
			{
				return false;
			}
		}
	}
	return true;
}

bool RaycastBuildTiledDropMeshDynamic::insectSolid(float* ps, float* pd, int x, int y)
{
	int minx, miny, maxx, maxy;
	rcHeightfield &testSolid = m_solidCache[y][x];
	if (testSolid.spans==0)
	{
		return false;
	}
	float t1[3];
	dtVsub(t1, ps, testSolid.bmin);
	minx = t1[0] / testSolid.cs;
	if (minx < 0)
		minx = 0;
	miny = t1[2] / testSolid.cs;
	if (miny < 0)
		miny = 0;
	dtVsub(t1, pd, testSolid.bmin);
	maxx = t1[0] / testSolid.cs;
	if (maxx >= testSolid.width)
		maxx = testSolid.width - 0.99f;
	maxy = t1[2] / testSolid.cs;
	if (maxy >= testSolid.height)
		maxy = testSolid.height - 0.99f;

	int cyMin = (ps[1] - testSolid.bmin[1]) / testSolid.ch + 3;
	int cyMax = cyMin + 2;

	for (int y = miny; y <= maxy; y++)
	{
		for (int x = minx; x <= maxx; x++)
		{
			rcSpan* span = testSolid.spans[testSolid.width*y + x];
			while (span)
			{
				if (!((span->smin < cyMin && span->smax < cyMin) || (span->smin > cyMax && span->smax > cyMax)))
				{
					return true;
				}
				span = span->next;
			}
		}
	}

	return false;
}

dtObstacleRef RaycastBuildTiledDropMeshDynamic::getObstacleRef(const frObstacle* ob) const
{
	if (!ob) return 0;
	const unsigned int idx = (unsigned int)(ob - m_obstacles);
	return encodeObstacleId(ob->salt, idx);
}

frObstacle* RaycastBuildTiledDropMeshDynamic::getObstacleByRef(dtObstacleRef ref)
{
	if (!ref)
		return 0;
	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_maxObstacles)
		return 0;
	frObstacle* ob = &m_obstacles[idx];
	unsigned int salt = decodeObstacleIdSalt(ref);
	if (ob->salt != salt)
		return 0;
	return ob;
}

void RaycastBuildTiledDropMeshDynamic::getObstacleBounds(const struct frObstacle* ob, float* bmin, float* bmax) const
{
	if (ob->type == DT_OBSTACLE_CYLINDER)
	{
		const dtObstacleCylinder &cl = ob->cylinder;

		bmin[0] = cl.pos[0] - cl.radius;
		bmin[1] = cl.pos[1];
		bmin[2] = cl.pos[2] - cl.radius;
		bmax[0] = cl.pos[0] + cl.radius;
		bmax[1] = cl.pos[1] + cl.height;
		bmax[2] = cl.pos[2] + cl.radius;
	}
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		dtVcopy(bmin, ob->box.bmin);
		dtVcopy(bmax, ob->box.bmax);
	}
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		const dtObstacleOrientedBox &orientedBox = ob->orientedBox;

		float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
		bmin[0] = orientedBox.center[0] - maxr;
		bmax[0] = orientedBox.center[0] + maxr;
		bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
		bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
		bmin[2] = orientedBox.center[2] - maxr;
		bmax[2] = orientedBox.center[2] + maxr;
	}
}

static bool contains(const dtCompressedTileRef* a, const int n, const dtCompressedTileRef v)
{
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return true;
	return false;
}

class updateTileCache
{
public:
	updateTileCache() {}
	~updateTileCache() {}

	void addTileCache(int x, int y)
	{
		bool finded = false;
		int tileIndex = y*MAX_TILEWIDTH + x;
		for (int j = 0; j < updateTilesNum; j++)
		{
			if (tileIndex == updateTiles[j])
			{
				finded = true;
				break;
			}
		}
		if (!finded)
		{
			updateTiles[updateTilesNum++] = tileIndex;
		}
	}
	bool hasTile(int x, int y)
	{
		int tileIndex = y*MAX_TILEWIDTH + x;
		for (int j = 0; j < updateTilesNum; j++)
		{
			if (tileIndex == updateTiles[j])
			{
				return true;
			}
		}
		return false;
	}
	void getPos(int index, int&x, int&y)
	{
		x = updateTiles[index] % MAX_TILEWIDTH;
		y = updateTiles[index] / MAX_TILEWIDTH;
	}
	int updateTiles[99];
	int updateTilesNum = 0;
	static const int MAX_TILEWIDTH = 10000;
};

dtStatus RaycastBuildTiledDropMeshDynamic::applyObstacle(int numNav, class dtNavMesh** navmeshes, int maxTile)
{
	if (maxTile <= 0)
	{
		maxTile = 99999;
	}
	dtStatus status = DT_SUCCESS;
	updateTileCache updateTiles;
	updateTileCache updateTilesBorder;
	for (int i = 0; i < m_nreqs; ++i)
	{
		ObstacleRequest* req = &m_reqs[i];
		frObstacle* ob = getObstacleByRef(req->ref);
		if (ob->state==DT_OBSTACLE_EMPTY)
		{
			continue;
		}

		if (req->action == REQUEST_REMOVE)
		{
			ob->next = m_nextFreeObstacle;
			m_nextFreeObstacle = ob;
		}
		float bmin[3], bmax[3];
		getObstacleBounds(ob, bmin, bmax);
		int ntouched = 0;

		const int tx0 = (int)dtMathFloorf((bmin[0] - m_BMin[0]) / m_tileWidth);
		const int tx1 = (int)dtMathFloorf((bmax[0] - m_BMin[0]) / m_tileWidth);
		const int ty0 = (int)dtMathFloorf((bmin[2] - m_BMin[2]) / m_tileHeight);
		const int ty1 = (int)dtMathFloorf((bmax[2] - m_BMin[2]) / m_tileHeight);

		for (int ty = ty0; ty <= ty1; ++ty)
		{
			for (int tx = tx0; tx <= tx1; ++tx)
			{
				if (tx<0 || ty<0 || tx>=m_widthTile || ty>=m_heightTile)
				{
					continue;
				}
				updateTiles.addTileCache(tx, ty);
			}
		}
		for (int ty = ty0-1; ty <= ty1+1; ++ty)
		{
			for (int tx = tx0-1; tx <= tx1+1; ++tx)
			{
				if (tx < 0 || ty < 0 || tx >= m_widthTile || ty >= m_heightTile)
				{
					continue;
				}
				if (!updateTiles.hasTile(tx, ty))
				{
					updateTilesBorder.addTileCache(tx, ty);
				}
			}
		}
	}
	m_nreqs = 0;
	for (int j=0; j<numNav; j++)
	{
		m_agentRadius = navmeshes[j]->getAgentRadius();
		for (int i = 0; i < updateTiles.updateTilesNum; i++)
		{
			int tx, ty;
			updateTiles.getPos(i, tx, ty);
			status = buildNavMeshTilesAt(tx, ty, navmeshes[j]);
			if (!dtStatusSucceed(status))
			{
				return status;
			}
		}
	}
	/*
	for (int i = 0; i < updateTilesBorder.updateTilesNum; i++)
	{
		int tx, ty;
		updateTilesBorder.getPos(i, tx, ty);
		status = genHeightField(tx, ty);
		if (!dtStatusSucceed(status))
		{
			return status;
		}
	}
	*/

	for (int j = 0; j < numNav; j++)
	{
		m_agentRadius = navmeshes[j]->getAgentRadius();
		for (int i = 0; i < updateTiles.updateTilesNum; i++)
		{
			int tx, ty;
			updateTiles.getPos(i, tx, ty);
			dtTileRef tileRef = navmeshes[j]->getTileRefAt(tx, ty, 0);
			const dtMeshTile* tile = navmeshes[j]->getTileByRef(tileRef);
			const dtNavMesh* nav = navmeshes[j];
			int tileIndex = tile - nav->getTile(0);
			int dataSize = 0;
			unsigned char* data = buildTileDropMesh(tileIndex, dataSize, navmeshes[j]);
			if (data)
			{
				navmeshes[j]->loadTileDropMesh(tileIndex, data, dataSize);
			}
			else
			{

			}
		}
	}

	//m_obstaclesInUse = 0;
	if (dtStatusSucceed(status))
	{
		return 0;
	}

	return status;
}



int RaycastBuildTiledDropMeshDynamic::assureInMap(float& x, float& y, float& z)
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