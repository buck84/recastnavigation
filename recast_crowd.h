#ifndef RECAST_CROWD_H_
#define RECAST_CROWD_H_

#include "DetourNavMesh.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCrowd.h"

struct CrowdToolParams
{
	bool m_expandSelectedDebugDraw;
	bool m_showCorners;
	bool m_showCollisionSegments;
	bool m_showPath;
	bool m_showVO;
	bool m_showOpt;
	bool m_showNeis;

	bool m_expandDebugDraw;
	bool m_showLabels;
	bool m_showGrid;
	bool m_showNodes;
	bool m_showPerfGraph;
	bool m_showDetailAll;

	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_optimizeVis;
	bool m_optimizeTopo;
	bool m_obstacleAvoidance;
	float m_obstacleAvoidanceType;
	bool m_separation;
	float m_separationWeight;
};

class RecastCrowd
{
	class NavMeshInstance* m_navmeshInterface;
	dtNavMesh* m_nav;
	dtCrowd* m_crowd;

	float m_targetPos[3];
	dtPolyRef m_targetRef;

	dtCrowdAgentDebugInfo m_agentDebug;
	dtObstacleAvoidanceDebugData* m_vod;

	static const int AGENT_MAX_TRAIL = 64;
	static const int MAX_AGENTS = 128;

	CrowdToolParams m_toolParams;

	bool m_run;

public:
	RecastCrowd();
	virtual ~RecastCrowd();

	void init(class NavMeshInstance* nmInterface);
	void reset();
	void handleUpdate(const float dt);

	inline bool isRunning() const { return m_run; }
	inline void setRunning(const bool s) { m_run = s; }

	int addAgent(const float* pos, float speed);
	void removeAgent(const int idx);
	void hilightAgent(const int idx);
	void updateAgentParams();
	void setMoveTarget(int idx, const float* p, bool adjust);
	void setSpeed(int idx, float s);
	void getPos(int idx, float* p);
	void updateTick(const float dt);

	inline CrowdToolParams* getToolParams() { return &m_toolParams; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	RecastCrowd(const RecastCrowd&);
	RecastCrowd& operator=(const RecastCrowd&);
};

#endif	// RECAST_CROWD_H_