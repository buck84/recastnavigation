#include "recast_build.h"
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
#include <float.h>


RaycastBuild::RaycastBuild() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_ctx(0),
	m_solidCache(0),
	m_tileWidth(0),
	m_tileHeight(0),
	m_widthTile(0),
	m_heightTile(0)

{
	resetCommonSettings();

	m_navQuery = dtAllocNavMeshQuery();
	m_ctx = new BuildContext();
}

void RaycastBuild::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_tileSize = 48,
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_dropHeight = 5.0f;
	m_dropHoriDistScale = 2.5f;
	m_regionMinSize = 2;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
}

int RaycastBuild::addMeshBegin()
{
	if (m_geom)
	{
		delete m_geom;
	}
	m_geom = new InputGeom;
	m_geom->reset(0);

	return 0;
}

int RaycastBuild::addMesh(int areaType, const float* vertices, const int vCount, const int* triangles, const int tCount)
{
	if (vCount != 0 && tCount != 0)
		m_geom->addMesh(areaType, vertices, vCount, triangles, tCount);

	return 0;
}

int RaycastBuild::addMeshEnd()
{
	return 0;
}

int RaycastBuild::exportMesh(std::string name)
{
	FILE* fp = fopen(name.c_str(), "wb");
	if (!fp)
		return 1;
	
	m_geom->writeFileVertical(fp);

	fclose(fp);
	return 0;
}

void RaycastBuild::clearOffmeshJumpedge()
{
	m_geom->clearOffmeshJumpedge();
}

void RaycastBuild::addOffMeshConnection(const float* spos, const float* epos, const float rad,
	unsigned char bidir, unsigned char area, unsigned short flags)
{
	m_geom->addOffMeshConnection(spos, epos, rad, bidir, area, flags);
}

int RaycastBuild::deleteOffMeshConnection(const float* pos)
{
	float nearestDist = FLT_MAX;
	int nearestIndex = -1;
	const float* verts = m_geom->getOffMeshConnectionVerts();
	for (int i = 0; i < m_geom->getOffMeshConnectionCount() * 2; ++i)
	{
		const float* v = &verts[i * 3];
		float d = rcVdistSqr(pos, v);
		if (d < nearestDist)
		{
			nearestDist = d;
			nearestIndex = i / 2; // Each link has two vertices.
		}
	}
	// If end point close enough, delete it.
	if (nearestIndex != -1 && sqrtf(nearestDist) < 1.0f)
	{
		m_geom->deleteOffMeshConnection(nearestIndex);
		return 0;
	}
	return 1;
}

int RaycastBuild::getOffMeshCount()
{
	if (m_geom)
	{
		return m_geom->getOffMeshConnectionCount();
	}
	return -1;
}

int RaycastBuild::getOffMeshData(float* p[])
{
	float*& outp = *p;
	if (m_geom)
	{
		const float* omvs = m_geom->getOffMeshConnectionVerts();
		for (int i = 0; i < 2 * 3 * m_geom->getOffMeshConnectionCount(); i++)
		{
			outp[i] = omvs[i];
		}
		return 0;
	}
	else
		return -1;
}
int RaycastBuild::addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret)
{
	return DT_SUCCESS;
}

int RaycastBuild::addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret)
{
	return DT_SUCCESS;
}

int RaycastBuild::removeObstacle(unsigned int ref)
{
	return DT_SUCCESS;
}

dtStatus RaycastBuild::applyObstacle(int numNav, class dtNavMesh** navmeshes, int maxTile)
{
	return DT_SUCCESS;
}

bool RaycastBuild::blockPlayer(float* s, float* e)
{
	if (!m_geom)
		return false;
	float t = -1.0f;
	m_geom->raycastMesh(s, e, t);
	return t > 0.0f && t < 1.0f;
}