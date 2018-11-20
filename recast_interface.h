#ifndef RECAST_MANAGER_H_
#define RECAST_MANAGER_H_

#include "recast_define.h"
#include "Recast.h"
#include "DetourNavMeshQuery.h"

#include <string>
#include <map>

class dtNavMesh;

class NavMeshInstance
{
private:

public:
	NavMeshInstance();
	~NavMeshInstance();
	float m_r;					/// agent radius
	float m_h = 1.8f;
	dtNavMesh* m_navMeshJump;	/// navmesh for jump test
	dtNavMesh* m_navMesh;		/// navmesh for move
	dtNavMeshQuery* m_navQueryJump;
	dtNavMeshQuery* m_navQuery;
	class RaycastBuild* m_builder;
	dtQueryFilter m_filter;
	class RecastCrowd* m_RecastCrowd;
	float queryExt[3];			/// query extent, inited in game, usually(m_r, 9999999, m_r)

	float m_jumpDistMax = 2.0f;	/// jump max distance in game, eg:window, barrier

	static const int MAX_POLYS = 256;	/// query parameter

	void navMeshLoaded();
	/// 游戏中移动，只是直线移动，其它跳跃需要逻辑触发
	int moveInGame(float* s, float* e, bool slide = false);
	/// 服务器跳跃逻辑
	int moveJumpServer(float* s, float* e);
	/// 客户端调用判断是否可以跳下，翻越...，显示相应按钮
	int moveNextType(float* s, float* e);

	int autoMove(float* s, float* e, float* path, int& pathNum);
	//int raycast(float* p1, float* p2, float& t);
	
	/// 最近点
	int getPosNear(float& x, float& y, float& z);	
	/// 高空跳下最高可降落点
	int getPosFall(float& x, float& y, float& z);
	/// 高于y的最小值
	int getPosCeil(float x, float& y, float z);
	/// 低于y的最大值
	int getPosFloor(float x, float& y, float z);
	/// 放置障碍以后重新计算位置
	int getPosObstAdjust(float& x, float& y, float& z);
	/// 所有高度
	int getHeights(float x, float y, float z, float* hs[], int& numH, int maxH);

	/// 获得以s为起点沿d方向前进l长度 将会经过的点
	int findMovePath(float* s, float* d, float l, float* path, int& pathNum, bool slide);

	/// poly normal, for obstacle object normal
	int getNormal(float x, float y, float z, float* n);
	
};

struct ObstacleInfo 
{
	unsigned int refMove;
	unsigned int refMoveJump;
};

class RecastInterface
{
public:
	RecastInterface();
	~RecastInterface();

	int Init(std::string name);
	int Init(const char* buf, int bufSize);
	int readBuffTiledDropMesh(const char* buf, int bufSize);
	int readBuffTiledDropMeshDynamic(const char* buf, int bufSize);
	int readBuffTempObst(const char* buf, int bufSize);
	int build();
	int update(float dt);

	int getAgentTypeNum() { return m_agentTypeNum; }
	NavMeshInstance* getNavMeshInstance(int agentType) { return m_instances + agentType; }

	//int move(int agentType, float* s, float* e, bool slide = true);
	int moveInGame(int agentType, float* s, float* e, bool slide = true);
	int moveJumpServer(int agentType, float* s, float* e);
	int autoMove(int agentType, float* s, float* e, float* path, int& pathNum);
	//int raycast(int agentType, float* p1, float* p2, float& t);
	int findMovePath(int agentType, float* s, float* d, float l, float* path, int& pathNum, bool slide = true);
	int getPosNear(int agentType, float& x, float& y, float& z);
	int getPosFall(int agentType, float& x, float& y, float& z);
	int getPosCeil(int agentType, float x, float& y, float z);
	int getPosFloor(int agentType, float x, float& y, float z);
	int getPosObstAdjust(int agentType, float& x, float& y, float& z);
	int getNormal(int agentType, float x, float& y, float z, float* n);
	int assureInMap(int agentType, float& x, float& y, float& z);

	int addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret, bool apply=true);
	int addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret, bool apply=true);
	int removeObstacle(unsigned int ref, bool apply = true);

	class RecastAgent* getAgent(int index);
	int addAgentIndex(int agentType, bool isPlayer, float* pos, float speed);
	class RecastAgent* addAgent(int agentType, bool isPlayer, float* pos, float speed);

	int setAgentTarget(int agentType, int idx, float* targetPos);
	int getAgentPos(int agentType, int idx, float* pos);

private:
	NavMeshInstance* m_instances;
	class RaycastBuild* m_builder;
	class RaycastBuild* m_builderJump;
	int m_agentTypeNum;

	int m_maxAgents;
	class RecastAgent* m_agents;

	float m_paramMaxSlop;
	float m_paramCellSize;
	float m_paramCellHeight;
	float m_paramDropHeight;
	float m_paramAgentHeight;
	float m_paramHDropScale;
	float m_paramTileSize;
	float m_paramStepHeight;
	float m_paramStepHeightJump;

	std::map<unsigned int, ObstacleInfo> m_obstacleInfo;

};
#endif	// RECAST_MANAGER_H_