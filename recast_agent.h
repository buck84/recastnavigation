#ifndef RECAST_AGENT_H_
#define RECAST_AGENT_H_

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class RecastInterface;

enum enumAgentClass
{
	AC_PLAYER,
	AC_AI,
};

enum enumAgentState
{
	AS_NORMAL,
	AS_FROZEN,
	AS_DEAD
};

class RecastAgent
{
public:
	RecastAgent();
	~RecastAgent() {}

	void setRecastInterface(RecastInterface *i) { m_recastInstance = i; }
	int init(int agentType, bool isPlayer, float *s);
	void dead() { m_state = AS_DEAD; }
	bool isActive() { return m_state != AS_DEAD; }

	int setPos(float x, float y, float z);
	const float* getPos() { return m_pos; }

	int update(float dt) { return 0;  }

	int setMoveTarget(float x, float y, float z);

private:
	class RecastInterface* m_recastInstance;
	int m_agentType;
	int m_class;
	int m_state;

	float m_pos[3];
	dtPolyRef m_PolyRef;
};
#endif	// RECAST_AGENT_H_