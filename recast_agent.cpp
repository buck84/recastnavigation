#include "recast_agent.h"
#include "recast_interface.h"
#include "Recast.h"
#include "DetourCommon.h"
#ifdef _FREE_SERVER_DEFINE
#include "Logger.h"
#endif

RecastAgent::RecastAgent() :
	m_state(AS_NORMAL)
{}

int RecastAgent::init(int agentType, bool isPlayer, float *s)
{
	m_agentType = agentType;
	m_class = isPlayer ? AC_PLAYER : AC_AI;

	dtVcopy(m_pos, s);
	return m_recastInstance->getPosNear(m_agentType, m_pos[0], m_pos[1], m_pos[2]);
}

int RecastAgent::setPos(float x, float y, float z)
{
	m_pos[0] = x;
	m_pos[1] = y; 
	m_pos[2] = z;
	return m_recastInstance->getPosNear(m_agentType, m_pos[0], m_pos[1], m_pos[2]);
}

int RecastAgent::setMoveTarget(float x, float y, float z)
{
	m_pos[0] = x;
	m_pos[1] = y;
	m_pos[2] = z;
	return m_recastInstance->getPosNear(m_agentType, m_pos[0], m_pos[1], m_pos[2]);
}