#include "recast_crowd.h"
#include "recast_interface.h"
#include "Recast.h"
#include "DetourCommon.h"
#ifdef _FREE_SERVER_DEFINE
#include "Logger.h"
#endif


RecastCrowd::RecastCrowd() :
	m_nav(0),
	m_crowd(0),
	m_targetRef(0),
	m_run(true)
{
	m_toolParams.m_expandSelectedDebugDraw = true;
	m_toolParams.m_showCorners = false;
	m_toolParams.m_showCollisionSegments = false;
	m_toolParams.m_showPath = false;
	m_toolParams.m_showVO = false;
	m_toolParams.m_showOpt = false;
	m_toolParams.m_showNeis = false;
	m_toolParams.m_expandDebugDraw = false;
	m_toolParams.m_showLabels = false;
	m_toolParams.m_showGrid = false;
	m_toolParams.m_showNodes = false;
	m_toolParams.m_showPerfGraph = false;
	m_toolParams.m_showDetailAll = false;
	m_toolParams.m_expandOptions = true;
	m_toolParams.m_anticipateTurns = true;
	m_toolParams.m_optimizeVis = true;
	m_toolParams.m_optimizeTopo = true;
	m_toolParams.m_obstacleAvoidance = true;
	m_toolParams.m_obstacleAvoidanceType = 3.0f;
	m_toolParams.m_separation = false;
	m_toolParams.m_separationWeight = 2.0f;

	m_vod = dtAllocObstacleAvoidanceDebugData();
	m_vod->init(2048);

	memset(&m_agentDebug, 0, sizeof(m_agentDebug));
	m_agentDebug.idx = -1;
	m_agentDebug.vod = m_vod;
}

RecastCrowd::~RecastCrowd()
{
	dtFreeObstacleAvoidanceDebugData(m_vod);
}

void RecastCrowd::init(class NavMeshInstance* nmInterface)
{
	m_navmeshInterface = nmInterface;
	m_crowd = dtAllocCrowd();

	dtNavMesh* nav = m_navmeshInterface->m_navMesh;
	dtCrowd* crowd = m_crowd;

	if (nav && crowd && (m_nav != nav || m_crowd != crowd))
	{
		m_nav = nav;
		m_crowd = crowd;

		crowd->init(MAX_AGENTS, m_navmeshInterface->m_r, nav);

		// Make polygons with 'disabled' flag invalid.
		crowd->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

		// Setup local avoidance params to different qualities.
		dtObstacleAvoidanceParams params;
		// Use mostly default settings, copy from dtCrowd.
		memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

		// Low (11)
		params.velBias = 0.5f;
		params.adaptiveDivs = 5;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 1;
		crowd->setObstacleAvoidanceParams(0, &params);

		// Medium (22)
		params.velBias = 0.5f;
		params.adaptiveDivs = 5;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 2;
		crowd->setObstacleAvoidanceParams(1, &params);

		// Good (45)
		params.velBias = 0.5f;
		params.adaptiveDivs = 7;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 3;
		crowd->setObstacleAvoidanceParams(2, &params);

		// High (66)
		params.velBias = 0.5f;
		params.adaptiveDivs = 7;
		params.adaptiveRings = 3;
		params.adaptiveDepth = 3;

		crowd->setObstacleAvoidanceParams(3, &params);
	}
}

void RecastCrowd::reset()
{
}

void RecastCrowd::handleUpdate(const float dt)
{
	if (m_run)
		updateTick(dt);
}

int RecastCrowd::addAgent(const float* p, float speed)
{
	dtCrowd* crowd = m_crowd;

	dtCrowdAgentParams ap;
	memset(&ap, 0, sizeof(ap));
	ap.radius = m_navmeshInterface->m_r;
	ap.height = m_navmeshInterface->m_h;
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = speed;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = 0;
	if (m_toolParams.m_anticipateTurns)
		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_toolParams.m_optimizeVis)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_toolParams.m_optimizeTopo)
		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_toolParams.m_obstacleAvoidance)
		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_separation)
		ap.updateFlags |= DT_CROWD_SEPARATION;
	ap.obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
	ap.separationWeight = m_toolParams.m_separationWeight;

	int idx = crowd->addAgent(p, &ap);
	if (idx != -1)
	{
		if (m_targetRef)
			crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
	}
	return idx;
}

void RecastCrowd::removeAgent(const int idx)
{
	if (!m_navmeshInterface) return;
	dtCrowd* crowd = m_crowd;

	crowd->removeAgent(idx);

	if (idx == m_agentDebug.idx)
		m_agentDebug.idx = -1;
}

void RecastCrowd::hilightAgent(const int idx)
{
	m_agentDebug.idx = idx;
}

static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

void RecastCrowd::setMoveTarget(int idx, const float* p, bool adjust)
{
	if (!m_navmeshInterface) return;

	dtCrowd* crowd = m_crowd;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (!ag || !ag->active)
		return;

	// Find nearest point on navmesh and set move request to that location.
	dtNavMeshQuery* navquery = m_navmeshInterface->m_navQuery;
	const dtQueryFilter* filter = crowd->getFilter(0);
	const float* halfExtents = crowd->getQueryExtents();

	if (adjust)
	{
		float vel[3];
		// Request velocity
		calcVel(vel, ag->npos, p, ag->params.maxSpeed);
		crowd->requestMoveVelocity(m_agentDebug.idx, vel);
	}
	else
	{
		navquery->findNearestPoly(p, halfExtents, filter, &m_targetRef, m_targetPos);
		crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
	}
}

void RecastCrowd::setSpeed(int idx, float s)
{
	const dtCrowdAgent* ag = m_crowd->getAgent(idx);
	if (!ag || !ag->active)
		return;

	dtCrowdAgentParams params = ag->params;
	params.maxSpeed = s;
	m_crowd->updateAgentParameters(idx, &params);
}

void RecastCrowd::getPos(int idx, float* p)
{
	if (!m_navmeshInterface) return;

	dtCrowd* crowd = m_crowd;
	const dtCrowdAgent* ag = crowd->getAgent(idx);
	if (!ag || !ag->active)
		return;

	memcpy(p, ag->npos, 3*sizeof(float));
}

void RecastCrowd::updateAgentParams()
{
	if (!m_navmeshInterface) return;
	dtCrowd* crowd = m_crowd;
	if (!crowd) return;

	unsigned char updateFlags = 0;
	unsigned char obstacleAvoidanceType = 0;

	if (m_toolParams.m_anticipateTurns)
		updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
	if (m_toolParams.m_optimizeVis)
		updateFlags |= DT_CROWD_OPTIMIZE_VIS;
	if (m_toolParams.m_optimizeTopo)
		updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_obstacleAvoidance)
		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
	if (m_toolParams.m_separation)
		updateFlags |= DT_CROWD_SEPARATION;

	obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;

	dtCrowdAgentParams params;

	for (int i = 0; i < crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = crowd->getAgent(i);
		if (!ag->active) continue;
		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
		params.updateFlags = updateFlags;
		params.obstacleAvoidanceType = obstacleAvoidanceType;
		params.separationWeight = m_toolParams.m_separationWeight;
		crowd->updateAgentParameters(i, &params);
	}
}

void RecastCrowd::updateTick(const float dt)
{
	if (!m_navmeshInterface) return;
	dtNavMesh* nav = m_navmeshInterface->m_navMesh;
	dtCrowd* crowd = m_crowd;
	if (!nav || !crowd) return;

	crowd->update(dt, &m_agentDebug);

	m_agentDebug.vod->normalizeSamples();
}
