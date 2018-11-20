#include "recast_interface.h"
#include "recast_build_tiled_dm.h"
#include "recast_build_tiled_dm_dynamic.h"
#include "recast_build_tempobstacle.h"
#include "recast_crowd.h"
#include "Recast.h"
#include "DetourCommon.h"
#include "recast_agent.h"
#include "assert.h"
#include <cstring>
#include <float.h>
#ifdef _FREE_SERVER_DEFINE
#include "Logger.h"
#endif

NavMeshInstance::NavMeshInstance() : 
	m_navMesh(0),
	m_navMeshJump(0),
	m_RecastCrowd(0)
{
	m_navQuery = dtAllocNavMeshQuery();
	m_navQueryJump = dtAllocNavMeshQuery();
}

NavMeshInstance::~NavMeshInstance()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);

	dtFreeNavMeshQuery(m_navQueryJump);
	dtFreeNavMesh(m_navMeshJump);
}

void NavMeshInstance::navMeshLoaded()
{
	if (m_RecastCrowd)
	{
		delete m_RecastCrowd;
	}
	m_RecastCrowd = new RecastCrowd();
	m_RecastCrowd->init(this);
}

int NavMeshInstance::moveInGame(float* s, float* e, bool slide)
{
	// 中间变量
	float hitPos[3];
	dtPolyRef polys[MAX_POLYS];
	dtRaycastHit hit;
	hit.path = polys;
	hit.maxPath = MAX_POLYS;
	// 找到当前poly，recast函数用
	dtPolyRef startRef;
	m_navQuery->findNearestPoly(s, queryExt, &m_filter, &startRef, 0);
	// 可能是因为碰撞被挤开了，需要找个最近位置
	if (startRef == 0)
	{
		int ret = getPosNear(s[0], s[1], s[2]);
		if (ret!=0)	// 找不到，卡住
		{
			e[0] = s[0];
			e[1] = s[1];
			e[2] = s[2];
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::moveInGame error1");
#endif
			return MR_NORMAL;
		}
	}
	// recast
	dtStatus ret = m_navQuery->raycast(startRef, s, e, &m_filter, 0, &hit);
	// 可能卡住走不了，取最近的可达点
	if (hit.t==0)
	{
		float clostest[3];
		dtVcopy(clostest, s);
		bool posOverlay=false;
		m_navQuery->closestPointOnPoly(startRef, s, clostest, &posOverlay);
		dtVcopy(s, clostest);
	}
	// move straight to
	if (hit.t >= 1)
	{
		float h = s[1];
		ret = m_navQuery->getPolyHeight(hit.path[hit.pathCount - 1], e, &h);
		if (dtStatusSucceed(ret))
		{
			e[1] = h;
		}
		return MR_NORMAL;
	}

	// Hit
	if (dtVlenSqr(hit.hitNormal) > 0)
	{
		dtVcopy(hitPos, hit.hitPos);
	}
	else // 容错，应该不会执行
	{
		dtVlerp(hitPos, s, e, hit.t);
		dtVcopy(e, hitPos);
		return MR_NORMAL;
	}

	if (slide)
	{
		float penetrate[3];
		dtVsub(penetrate, e, hitPos);
		float hitTang[3];
		hitTang[0] = hit.hitNormal[2];
		hitTang[1] = 0;
		hitTang[2] = -hit.hitNormal[0];
		dtVnormalize(hitTang);
		// calc slide dist
		float dist = dtVdot(hitTang, penetrate);
		dtVscale(hitTang, hitTang, dist);
		// calc slide move dest position
		float slideDst[3];
		dtVadd(slideDst, hitPos, hitTang);
		// 向外偏移一定距离计算，消除共线误差
		float normalCorrect[3];
		dtVscale(normalCorrect, hit.hitNormal, 0.001f);
		dtVadd(slideDst, slideDst, normalCorrect);
		// recast
		ret = m_navQuery->raycast(startRef, s, slideDst, &m_filter, 0, &hit);// &t, hitNormal, polys, &npolys, MAX_POLYS, &drop);
		
		if (hit.t >= 1)
		{
			float h = s[1];
			ret = m_navQuery->getPolyHeight(hit.path[hit.pathCount - 1], slideDst, &h);
			if (dtStatusSucceed(ret))
			{
				slideDst[1] = h;
			}
			dtVcopy(e, slideDst);
		}
		else
		{
			if (dtVlenSqr(hit.hitNormal) > 0)
			{
				dtVcopy(e, hit.hitPos);
			}
			else // 容错，应该不会执行
			{
				dtVlerp(e, s, slideDst, hit.t);
			}
		}
	}
	else
	{
		dtVcopy(e, hitPos);
	}
	
	return MR_TODO;
}

int NavMeshInstance::moveJumpServer(float* s, float* e)
{
	float nearestPt[3];
	float hitNormal[3];
	float hitPos[3];
	dtPolyRef polys[MAX_POLYS];
	dtPolyRef startRef;
	m_navQuery->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
	float t = 0;
	int npolys = 0;
	int drop;
	dtStatus ret = m_navQuery->raycast(startRef, s, e, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS, &drop);
	// move straight to
	if (t >= 1)
	{
		// No hit
		m_navQuery->findNearestPoly(e, queryExt, &m_filter, &startRef, nearestPt);
		dtVcopy(e, nearestPt);
		return MR_NORMAL;
	}

	// Hit
	dtVlerp(hitPos, s, e, t);
	float dir[3];
	dtVsub(dir, e, s);
	dtVnormalize(dir);
	dtVscale(dir, dir, m_jumpDistMax);
	float dst[3];
	dtVadd(dst, hitPos, dir);
	dtPolyRef dstPolyRef;
	float hitPosJump[3];
	float jumpt = 1.0f;
	ret = m_navQuery->moveJump(0, hitPos, dst, &m_filter, &jumpt, hitPosJump, hitNormal, &dstPolyRef);

	const float climbHeightMax = m_navMeshJump->getClimbHeight();
	const float climbHeightMin = 0.5f;
	const float jumpHeightMin = -0.3f;
	const float dropSafeHeight = -m_navMesh->getJumpSafeHeight();

	if (jumpt < 1 && jumpt>0)
	{
		float moveDeltaHeight = hitPosJump[1] - s[1];
		if (moveDeltaHeight > climbHeightMax)			// blocked
		{
			dtVcopy(e, hitPos);
			return MR_BLOCKED;
		}
		else if (moveDeltaHeight > jumpHeightMin)	// climb&jump
		{
			m_navQueryJump->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
			ret = m_navQueryJump->raycast(startRef, s, e, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS);

			if (t >= 1)
			{
				dtVcopy(e, hitPosJump);
				if (moveDeltaHeight > climbHeightMin)
					return MR_CLIMB;
				else
					return MR_JUMP;
			}
			dtVcopy(e, hitPos);
			return MR_BLOCKED;
		}
		else // drop
		{
			float testDrops[3];
			testDrops[0] = s[0];
			testDrops[1] = s[1] + 0.25f;
			testDrops[2] = s[2];
			float testDrope[3];
			testDrope[0] = hitPosJump[0];
			testDrope[1] = testDrops[1];
			testDrope[2] = hitPosJump[2];
			if (m_builder->blockPlayer(testDrops, testDrope))
			{
				dtVcopy(e, hitPos);
				return MR_BLOCKED;
			}
			else
			{
				dtVcopy(e, hitPosJump);
				if (moveDeltaHeight > dropSafeHeight)
					return MR_DROP_LOW;
				else
					return MR_DROP_HIGH;
			}
		}
	}

	/*
	m_navQueryJump->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
	ret = m_navQueryJump->raycast(startRef, s, e, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS);
	if (t >= 1)
	{
		float dir[3];
		dtVsub(dir, e, s);
		dtVnormalize(dir);
		dtVscale(dir, dir, m_jumpDistMax);
		float dst[3];
		dtVadd(dst, hitPos, dir);
		dtPolyRef dstPolyRef;
		float hitPosJump[3];
		float jumpt = 1.0f;
		ret = m_navQuery->moveJump(0, hitPos, dst, &m_filter, &jumpt, hitPosJump, hitNormal, &dstPolyRef);
		if (jumpt < 1 && jumpt>0)
		{
			dtVcopy(e, hitPosJump);
			if (hitPosJump[1] - s[1] > 0.5f)
			{
				return MR_CLIMB;
			}
			else
				return MR_JUMP;
		}
		else
		{
			dtVcopy(e, hitPos);
			return MR_BLOCKED;
		}
	}
	*/
	dtVcopy(e, hitPos);
	return MR_BLOCKED;
}

int NavMeshInstance::moveNextType(float* s, float* e)
{
	float nearestPt[3];
	float hitNormal[3];
	float hitPos[3];
	dtPolyRef polys[MAX_POLYS];
	dtPolyRef startRef;
	m_navQuery->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
	float t = 0;
	int npolys = 0;
	int drop;
	dtStatus ret = m_navQuery->raycast(startRef, s, e, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS, &drop);
	// move straight to
	if (t >= 1)
	{
		// No hit
		return MR_NORMAL;
	}
	// Hit
	dtVlerp(hitPos, s, e, t);
	float dir[3];
	dtVsub(dir, e, s);
	dtVnormalize(dir);
	dtVscale(dir, dir, m_jumpDistMax);
	float dst[3];
	dtVadd(dst, hitPos, dir);
	dtPolyRef dstPolyRef;
	float hitPosJump[3];
	float jumpt = 1.0f;
	ret = m_navQuery->moveJump(0, hitPos, dst, &m_filter, &jumpt, hitPosJump, hitNormal, &dstPolyRef);

	const float climbHeightMax = m_navMeshJump->getClimbHeight();
	const float climbHeightMin = 0.5f;
	const float jumpHeightMin = -0.3f;
	const float dropSafeHeight = -m_navMesh->getJumpSafeHeight();

	if (jumpt < 1 && jumpt>0)
	{
		float moveDeltaHeight = hitPosJump[1] - s[1];
		if (moveDeltaHeight>climbHeightMax)			// blocked
		{
			return MR_BLOCKED;
		}
		else if (moveDeltaHeight > jumpHeightMin)	// climb&jump
		{
			m_navQueryJump->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
			ret = m_navQueryJump->raycast(startRef, s, e, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS);

			if (t >= 1)
			{
				if (moveDeltaHeight > climbHeightMin)
					return MR_CLIMB;
				else
					return MR_JUMP;
			}
			return MR_BLOCKED;
		}
		else // drop
		{
			float testDrops[3];
			testDrops[0] = s[0];
			testDrops[1] = s[1] + 0.25f;
			testDrops[2] = s[2];
			float testDrope[3];
			testDrope[0] = hitPosJump[0];
			testDrope[1] = testDrops[1];
			testDrope[2] = hitPosJump[2];
			if (m_builder->blockPlayer(testDrops, testDrope))
				return MR_BLOCKED;
			else
			{
				if (moveDeltaHeight > dropSafeHeight)
					return MR_DROP_LOW;
				else
					return MR_DROP_HIGH;
			}
		}
	}
	/*
	if (t >= 1)
	{
		float dir[3];
		dtVsub(dir, e, s);
		dtVnormalize(dir);
		dtVscale(dir, dir, m_jumpDistMax);
		float dst[3];
		dtVadd(dst, hitPos, dir);
		dtPolyRef dstPolyRef;
		float hitPosJump[3];
		float jumpt = 1.0f;
		ret = m_navQuery->moveJump(0, hitPos, dst, &m_filter, &jumpt, hitPosJump, hitNormal, &dstPolyRef);
		if (jumpt < 1 && jumpt>0)
		{
			if (hitPosJump[1] - s[1] > 0.5f)
			{
				return MR_CLIMB;
			}
			else
				return MR_JUMP;
		}
		else
		{
			return MR_BLOCKED;
		}
	}
	*/
	return MR_BLOCKED;
}

int NavMeshInstance::autoMove(float* s, float* e, float* path, int& pathNum)
{
	if (pathNum<=0)
	{
		return 4;
	}
	if (pathNum > MAX_POLYS)
	{
		return 5;
	}
	float nearestPt[3];
	float spos[3];
	float epos[3];
	dtPolyRef startRef;
	dtPolyRef endRef;
	m_navQuery->findNearestPoly(s, queryExt, &m_filter, &startRef, nearestPt);
	if (startRef == 0)
	{
		int ret = getPosNear(s[0], s[1], s[2]);
		if (ret != 0)	// 找不到，卡住
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::autoMove error1");
#endif
			return 1;
		}
	}
	dtVcopy(spos, nearestPt);
	m_navQuery->findNearestPoly(e, queryExt, &m_filter, &endRef, nearestPt);
	if (endRef == 0)
	{
		int ret = getPosNear(e[0], e[1], e[2]);
		if (ret != 0)	// 找不到，卡住
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::autoMove error2");
#endif
			return 2;
		}
	}
	dtVcopy(epos, nearestPt);
	dtPolyRef* polys = new dtPolyRef[pathNum];
	int npolys;
	m_navQuery->findPath(startRef, endRef, spos, epos, &m_filter, polys, &npolys, pathNum);
	if (npolys)
	{
		float straightPath[MAX_POLYS * 3];
		unsigned char straightPathFlags[MAX_POLYS];
		dtPolyRef straightPathPolys[MAX_POLYS];
		int nstraightPath;
		dtStatus ret = m_navQuery->findStraightPath(spos, epos, polys, npolys,
			straightPath, straightPathFlags,
			straightPathPolys, &nstraightPath, MAX_POLYS, 0);

		if (nstraightPath<pathNum)
		{
			pathNum = nstraightPath;
		}
		//for(int i=0; i<nstraightPath*3; i++)
		//	(*path)[i] = straightPath[i];
		memcpy(path, straightPath, pathNum * 3 * sizeof(float));
	}
	else
		return 3;

	return 0;
}
/*
int NavMeshInstance::raycast(float* p1, float* p2, float& t)
{
	t = 0;
	float nearestPt[3];
	float spos[3];
	float epos[3];
	dtPolyRef startRef;
	dtPolyRef endRef;
	m_navQuery->findNearestPoly(p1, queryExt, &m_filter, &startRef, nearestPt);
	if (startRef == 0)
	{
#ifdef _FREE_SERVER_DEFINE
		ERR_LOG("NavMeshInstance::raycast startRef error");
#endif
		return 1;
	}
	dtVcopy(spos, nearestPt);
	m_navQuery->findNearestPoly(p2, queryExt, &m_filter, &endRef, nearestPt);
	if (endRef == 0)
	{
#ifdef _FREE_SERVER_DEFINE
		ERR_LOG("NavMeshInstance::raycast endRef error");
#endif
		return 2;
	}
	dtVcopy(epos, nearestPt);
	float hitNormal[3];
	dtPolyRef polys[MAX_POLYS];
	int npolys;
	dtStatus ret = m_navQuery->raycast(startRef, spos, epos, &m_filter, &t, hitNormal, polys, &npolys, MAX_POLYS);
	return 0;
}
*/
int NavMeshInstance::getPosNear(float& x, float& y, float& z)
{
	/*
	if (x < -1000 || x>1000 || y < -1000 || y>1000 || z < -1000 || z>1000)
	{
		int a = 3;
		a = 4;
	}
	*/
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float polyPickExt[3];
	dtVcopy(polyPickExt, queryExt);

	do 
	{
		float nearestPt[3];
		dtPolyRef polyRef = 0;
		dtStatus ret = m_navQuery->findNearestPoly(pos, polyPickExt, &m_filter, &polyRef, nearestPt);
		if (dtStatusSucceed(ret) && polyRef!=0)
		{
			x = nearestPt[0];
			y = nearestPt[1];
			z = nearestPt[2];
			return 0;
		}

		dtVscale(polyPickExt, polyPickExt, 2);
	} while (polyPickExt[0]<10.0f);

#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::getPosNear error:(%f,%f,%f)", x, y, z);
#endif
	return 10;
}

int NavMeshInstance::getPosFall(float& x, float& y, float& z)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float ext[3] = { 0.3f, 2.0f, 0.3f };
	float posTest[3];
	dtPolyRef prs[5];
	int prCount = 0;
	bool overlay = false;
	float besty = -FLT_MAX;
	bool hasOverlay = false;

	dtStatus ret = m_navQuery->queryPolygons(pos, ext, &m_filter, prs, &prCount, 5);
	if (dtStatusFailed(ret))
	{
#ifdef _FREE_SERVER_DEFINE
		ERR_LOG("NavMeshInstance::getPosFall queryPolygons error");
#endif
		return 1;
	}
	if (prCount==0)
	{
		return 2;
	}
	int retCode = 3;
	for (int i = 0; i < prCount; i++)
	{
		ret = m_navQuery->closestPointOnPoly(prs[i], pos, posTest, &overlay);
		if (dtStatusFailed(ret))
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::getPosFall closestPointOnPoly error");
#endif
			continue;
		}
		if (overlay && posTest[1] > besty)
		{
			besty = posTest[1];
			x = posTest[0];
			y = posTest[1];
			z = posTest[2];
			retCode = 0;
		}
	}
	return retCode;
}

int NavMeshInstance::getPosCeil(float x, float& y, float z)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float posTest[3];
	dtPolyRef prs[5];
	int prCount = 0;
	bool overlay = false;
	float besty = FLT_MAX;

	dtStatus ret = m_navQuery->queryPolygons(pos, queryExt, &m_filter, prs, &prCount, 5);
	if (dtStatusSucceed(ret))
	{
		for (int i = 0; i < prCount; i++)
		{
			m_navQuery->closestPointOnPoly(prs[i], pos, posTest, &overlay);
			if (overlay && posTest[1] >= y)
			{
				if (posTest[1] < besty)
				{
					besty = posTest[1];
					y = besty;
				}
			}
		}
		return 0;
	}
#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::getPosCeil error");
#endif
	return 1;
}

int NavMeshInstance::getPosFloor(float x, float& y, float z)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float posTest[3];
	dtPolyRef prs[5];
	int prCount = 0;
	bool overlay = false;
	float besty = -FLT_MAX;

	dtStatus ret = m_navQuery->queryPolygons(pos, queryExt, &m_filter, prs, &prCount, 5);
	if (dtStatusSucceed(ret))
	{
		for (int i = 0; i < prCount; i++)
		{
			m_navQuery->closestPointOnPoly(prs[i], pos, posTest, &overlay);
			if (overlay && posTest[1] <= y)
			{
				if (posTest[1] > besty)
				{
					besty = posTest[1];
					y = besty;
				}
			}
		}
		return 0;
	}
#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::getPosFloor error");
#endif
	return 1;
}

int NavMeshInstance::getPosObstAdjust(float& x, float& y, float& z)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float posTest[3];
	dtPolyRef prs[5];
	int prCount = 0;
	bool overlay = false;
	float besty = FLT_MAX;

	dtStatus ret = m_navQuery->queryPolygons(pos, queryExt, &m_filter, prs, &prCount, 5);
	if (dtStatusFailed(ret))
	{
#ifdef _FREE_SERVER_DEFINE
		ERR_LOG("NavMeshInstance::getPosObstAdjust error 1");
#endif
		return 1;

	}
	bool adjustOk = false;
	for (int i = 0; i < prCount; i++)
	{
		m_navQuery->closestPointOnPoly(prs[i], pos, posTest, &overlay);
		if (overlay && posTest[1] >= y)
		{
			if (posTest[1] < besty)
			{
				besty = posTest[1];
				y = besty;
			}
			adjustOk = true;
		}
	}

	if (!adjustOk)
	{
		int ret = getPosNear(pos[0], pos[1], pos[2]);
		if (ret!=0)
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::getPosObstAdjust error 2");
#endif
			return ret;
		}
		x = pos[0];
		y = pos[1];
		z = pos[2];
	}

	return 0;
}

int NavMeshInstance::getHeights(float x, float y, float z, float* hs[], int& numH, int maxH)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;
	float posTest[3];
	dtPolyRef prs[5];
	int prCount = 0;
	bool overlay = false;
	int retNum = 0;

	dtStatus ret = m_navQuery->queryPolygons(pos, queryExt, &m_filter, prs, &prCount, 5);
	if (dtStatusFailed(ret))
	{
#ifdef _FREE_SERVER_DEFINE
		ERR_LOG("NavMeshInstance::getHeights error");
#endif
		numH = 0;
		return ret;
	}
	for (int i = 0; i < prCount; i++)
	{
		m_navQuery->closestPointOnPoly(prs[i], pos, posTest, &overlay);
		if (overlay)
		{
			(*hs)[retNum] = posTest[1];
			retNum++;
			if (retNum>=maxH)
			{
				break;
			}
		}
	}
	numH = retNum;
	return 0;
}

int NavMeshInstance::findMovePath(float* p, float* d, float l, float* path, int& pathNum, bool slide)
{
	pathNum = 0;

	float s[3];
	dtVcopy(s, p);
	
	dtPolyRef polys[MAX_POLYS];
	dtRaycastHit hit;
	hit.path = polys;
	hit.maxPath = MAX_POLYS;

	dtPolyRef startRef;
	m_navQuery->findNearestPoly(s, queryExt, &m_filter, &startRef, s);

	int iterCount = 0;
	while (++iterCount<20)
	{
		// 已经找到终点
		if (startRef==0)
		{
			break;
		}
		float e[3];
		e[0] = s[0] + d[0] * l;
		e[1] = s[1] + d[1] * l;
		e[2] = s[2] + d[2] * l;

		dtStatus ret = m_navQuery->raycast(startRef, s, e, &m_filter, 0, &hit);
		if (dtStatusFailed(ret))
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::findMovePath RECAST1 error ret=%x s:(%f,%f,%f) e:(%f,%f,%f)", ret, s[0], s[1], s[2], e[0], e[1], e[2]);
#endif
		}
		if (hit.drop>0)
		{
			hit.pathCount = hit.pathCount - 1;
		}
		float t = hit.t;
		// 可以直达
		if (hit.t>=1.0f)
		{
			t = 1.0f;
			float h = s[1];
			ret = m_navQuery->getPolyHeight(hit.path[hit.pathCount - 1], e, &h);
			if (dtStatusSucceed(ret))
			{
				e[1] = h;
			}
			dtVcopy(path + pathNum * 3, e);
			if (e[0] > 10000 || e[0] < -10000)
			{
				int a = 3;
				a = 4;
			}
			pathNum += 1;
			break;
		}
		else
		{
			if (dtVlenSqr(hit.hitNormal) > 0)
			{
				dtVcopy(e, hit.hitPos);
			}
			else
				dtVlerp(e, s, e, hit.t);
		}

		// 离墙足够远，增加第一个通过点
		if (t > 0.01)
		{
			dtVcopy(path + pathNum * 3, e);
			if (e[0] > 10000 || e[0] < -10000)
			{
				int a = 3;
				a = 4;
			}
			pathNum += 1;
		}

		if (!slide)
			break;

		float dist = dtVdist(s, e);
		if (dist < 0)
			dist = -dist;
		l -= dist;		
		if (l <= 0.01)
			break;

		// 考虑浮点数计算精度，切向移动偏移一定距离
		float normalCorrect[3];
		dtVscale(normalCorrect, hit.hitNormal, 0.01f);

		// 计算切线
		float hitTang[3];
		hitTang[0] = hit.hitNormal[2];
		hitTang[1] = 0;
		hitTang[2] = -hit.hitNormal[0];
		dist = dtVdot(hitTang, d) * l;

		// 撞墙不能侧向滑动，直接返回
		if (dist>=-0.01 && dist <= 0.01)
			break;
		
		// 设置起始终止点
		dtVadd(s, e, normalCorrect);
		dtVmad(e, s, hitTang, dist);

		// 切线方向recast
		startRef = hit.path[hit.pathCount - 1];
		ret = m_navQuery->raycast(startRef, s, e, &m_filter, 0, &hit);
		if (hit.drop > 0)
		{
			hit.pathCount = hit.pathCount - 1;
		}

		if (dtStatusFailed(ret))
		{
#ifdef _FREE_SERVER_DEFINE
			ERR_LOG("NavMeshInstance::findMovePath RECAST2 error ret=%x s:(%f,%f,%f) e:(%f,%f,%f)", ret, s[0], s[1], s[2], e[0], e[1], e[2]);
#endif
		}
		t = hit.t;
		if (t < 0.001f)
		{
			//path[pathNum * 3 + 0] = e[0];
			//path[pathNum * 3 + 1] = e[1];
			//path[pathNum * 3 + 2] = e[2];
			//pathNum += 1;
			break;
		}
		if (hit.pathCount == 0)
		{
			break;
		}
		if (hit.t>1)
		{
			t = 1.0f;
			float h = s[1];
			ret = m_navQuery->getPolyHeight(hit.path[hit.pathCount - 1], e, &h);
			if (dtStatusSucceed(ret))
			{
				e[1] = h;
			}
		}
		else
		{
			if (dtVlenSqr(hit.hitNormal) > 0)
			{
				dtVcopy(e, hit.hitPos);
			}
			else
				dtVlerp(e, s, e, hit.t);
		}

		// recast与当前poly相交，继续查找
		if (hit.pathCount == 1)
		{
			dtVcopy(path + pathNum * 3, e);
			if (e[0] > 10000 || e[0] < -10000)
			{
				int a = 3;
				a = 4;
			}
			pathNum += 1;
			dtVcopy(s, e);
			continue;
		}

		/*
		*/

		// 查找接下来2个poly的相交点，因为可能滑动提前结束继续前进
		float straightPath[6];
		unsigned char straightFlags[2];
		dtPolyRef straightPolys[2];
		int straightPathCount = 0;
		ret = m_navQuery->findStraightPath(
			s, e, hit.path, hit.pathCount, straightPath, straightFlags,
			straightPolys, &straightPathCount, 2, DT_STRAIGHTPATH_ALL_CROSSINGS);

		e[0] = straightPath[3];
		e[1] = straightPath[4];
		e[2] = straightPath[5];
		float h = 0;
		ret = m_navQuery->getPolyHeight(straightPolys[1], e, &h);
		if (dtStatusSucceed(ret))
		{
			e[1] = h;
		}
		else
		{
			int a = 3;
			a = 4;
		}

		dist = dtVdist(s, e) / dtVdot(hitTang, d);

		dtVcopy(path + pathNum * 3, e);
		if (e[0] > 10000 || e[0] < -10000)
		{
			int a = 3;
			a = 4;
		}
		pathNum += 1;

		if (dist < 0)
			dist = -dist;
		l -= dist;

		if (l <= 0.01)
			break;

		dtVcopy(s, e);
		startRef = straightPolys[straightPathCount - 1];
	}

	return 0;
}

int NavMeshInstance::getNormal(float x, float y, float z, float* n)
{
	float pos[3];
	pos[0] = x;
	pos[1] = y;
	pos[2] = z;

	float nearestPt[3];
	dtPolyRef polyRef = 0;
	dtStatus ret = m_navQuery->findNearestPoly(pos, queryExt, &m_filter, &polyRef, nearestPt);
	if (dtStatusFailed(ret))
		return ret;
	if (polyRef == 0)
		return 1;

	m_navQuery->getPolyNormal(polyRef, nearestPt, n);
	return 0;
}


RecastInterface::RecastInterface() :
	m_instances(0),
	m_builder(0),
	m_builderJump(0),
	m_maxAgents(100),
	m_agents(0)
{
	m_agents = new RecastAgent[m_maxAgents];
	for (int i=0; i<m_maxAgents; i++)
	{
		m_agents[i].setRecastInterface(this);
	}
	m_builder = new RaycastBuildTempObstacle();
	m_builderJump = new RaycastBuildTempObstacle();
}

RecastInterface::~RecastInterface()
{
	if (m_instances)
	{
		delete[] m_instances;
		m_instances = 0;
	}
	if (m_builder)
	{
		delete m_builder;
		m_builder = 0;
	}
	if (m_builderJump)
	{
		delete m_builderJump;
		m_builderJump = 0;
	}
	if (m_agents)
	{
		delete[]m_agents;
		m_agents = 0;
	}
}

const int MAGIC_CODE_NAV = 'n' | 'a' << 8 | 'v' << 16 | 'm' << 24;
const int MAGIC_CODE_NAVJUMP = 'j' | 'u' << 8 | 'm' << 16 | 'p' << 24;

int RecastInterface::Init(std::string name)
{
	FILE* fp = fopen(name.c_str(), "rb");
	if (!fp)
		return 1;

	fseek(fp, 0, SEEK_END);
	int bufSize = ftell(fp);
	char* buf = new char[bufSize];
	fseek(fp, 0, SEEK_SET);
	fread(buf, bufSize, 1, fp);
	int ret = Init(buf, bufSize);

	delete[] buf;
	fclose(fp);
	return ret;
}

int RecastInterface::Init(const char* buf, int bufSize)
{
	int ret = 0;
	do 
	{
		int magicCode;
		readBuffer(&buf, bufSize, &magicCode, sizeof(int));
		if (magicCode != MAGIC_CODE_NAV)
		{
			ret = 2;
			break;
		}
		int nmType;
		readBuffer(&buf, bufSize, &nmType, sizeof(int));
		if (nmType == NM_TILED_DROPMESH)
		{
			ret = readBuffTiledDropMesh(buf, bufSize);
		}
		else if (nmType == NM_TILED_DROPMESH_DYNAMIC)
		{
			ret = readBuffTiledDropMeshDynamic(buf, bufSize);
		}
		else if (nmType == NM_TEMPOBST)
		{
			ret = readBuffTempObst(buf, bufSize);
		}
		else
			ret = 10000 + nmType;
	} while (false);

	return ret;
}

int RecastInterface::readBuffTiledDropMesh(const char* buf, int bufSize)
{
	if (m_builder)
	{
		delete m_builder;
	}
	m_builder = new RaycastBuildTiledDropMesh();

	readBuffer(&buf, bufSize, &m_agentTypeNum, sizeof(int));
	m_instances = new NavMeshInstance[m_agentTypeNum];
	for (int i = 0; i < m_agentTypeNum; i++)
	{
		int dataLen = 0;
		char* data;
		float r;
		int magicCode;

		if (!readBuffer(&buf, bufSize, &r, sizeof(int)))
			return 11;
		m_instances[i].m_r = r;

		if (!readBuffer(&buf, bufSize, &magicCode, sizeof(int)))
			return 12;
		if (magicCode != MAGIC_CODE_NAVJUMP)
		{
			return 13;
		}
		if (!readBuffer(&buf, bufSize, &dataLen, sizeof(int)))
			return 14;
		data = new char[dataLen];
		if (!readBuffer(&buf, bufSize, data, dataLen))
		{
			delete[] data;
			return 15;
		}
		m_instances[i].m_navMeshJump = m_builder->loadAll(data, dataLen);
		delete[] data;
		if (!readBuffer(&buf, bufSize, &dataLen, sizeof(int)))
			return 16;
		data = new char[dataLen];
		if (!readBuffer(&buf, bufSize, data, dataLen))
		{
			delete[] data;
			return 17;
		}
		m_instances[i].m_navMesh = m_builder->loadAll(data, dataLen);
		delete[] data;

		m_instances[i].m_navQueryJump->init(m_instances[i].m_navMeshJump, 2048);
		m_instances[i].m_navQuery->init(m_instances[i].m_navMesh, 2048);
		m_instances[i].queryExt[0] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].queryExt[1] = 1500.0f;
		m_instances[i].queryExt[2] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].navMeshLoaded();
	}

	int dataSize = 0;
	readBuffer(&buf, bufSize, &dataSize, sizeof(int));
	m_builder->addMeshBegin();
	int numMesh;
	int numV;
	int numT;
	readBuffer(&buf, bufSize, &numMesh, sizeof(int));
	for (int i = 0; i < numMesh; i++)
	{
		readBuffer(&buf, bufSize, &numV, sizeof(int));
		const float* vertices = (float*)buf;
		buf += numV * 3 * sizeof(float);
		readBuffer(&buf, bufSize, &numT, sizeof(int));
		const int* triangles = (int*)buf;
		buf += numT * 3 * sizeof(int);
		m_builder->addMesh(0, vertices, numV, triangles, numT);
	}
	m_builder->addMeshEnd();

	return 0;
}

int RecastInterface::readBuffTiledDropMeshDynamic(const char* buf, int bufSize)
{
	if (m_builder)
	{
		delete m_builder;
	}
	m_builder = new RaycastBuildTiledDropMeshDynamic();
	int magicCode;
	readBuffer(&buf, bufSize, &magicCode, sizeof(int));
	if (magicCode != MAGIC_CODE_NAV)
	{
		return 2;
	}

	readBuffer(&buf, bufSize, &m_agentTypeNum, sizeof(int));
	m_instances = new NavMeshInstance[m_agentTypeNum];
	for (int i = 0; i < m_agentTypeNum; i++)
	{
		float r;

		if (!readBuffer(&buf, bufSize, &r, sizeof(int)))
			return 11;
		m_instances[i].m_r = r;
	}

	readBuffer(&buf, bufSize, &m_paramMaxSlop, sizeof(float));
	m_builder->setAgentMaxSlope(m_paramMaxSlop);
	readBuffer(&buf, bufSize, &m_paramCellSize, sizeof(float));
	m_builder->setCellSize(m_paramCellSize);
	readBuffer(&buf, bufSize, &m_paramCellHeight, sizeof(float));
	m_builder->setCellHeight(m_paramCellHeight);
	readBuffer(&buf, bufSize, &m_paramDropHeight, sizeof(float));
	m_builder->setDropHeightAutoMove(m_paramDropHeight);
	readBuffer(&buf, bufSize, &m_paramAgentHeight, sizeof(float));
	m_builder->setAgentHeight(m_paramAgentHeight);
	readBuffer(&buf, bufSize, &m_paramHDropScale, sizeof(float));
	m_builder->setDropHoriDistScale(m_paramHDropScale);
	readBuffer(&buf, bufSize, &m_paramTileSize, sizeof(float));
	m_builder->setTileSize(m_paramTileSize);

	readBuffer(&buf, bufSize, &m_paramStepHeight, sizeof(float));
	readBuffer(&buf, bufSize, &m_paramStepHeightJump, sizeof(float));

	m_builder->addMeshBegin();
	int numMesh;
	int numV;
	int numT;
	readBuffer(&buf, bufSize, &numMesh, sizeof(int));
	for (int i=0; i<numMesh; i++)
	{
		readBuffer(&buf, bufSize, &numV, sizeof(int));
		const float* vertices = (float*)buf;
		buf += numV * 3 * sizeof(float);
		readBuffer(&buf, bufSize, &numT, sizeof(int));
		const int* triangles = (int*)buf;
		buf += numT * 3 * sizeof(int);
		m_builder->addMesh(0, vertices, numV, triangles, numT);
	}
	m_builder->addMeshEnd();

	return 0;
}

int RecastInterface::readBuffTempObst(const char* buf, int bufSize)
{
	readBuffer(&buf, bufSize, &m_agentTypeNum, sizeof(int));
	m_instances = new NavMeshInstance[m_agentTypeNum];
	for (int i = 0; i < m_agentTypeNum; i++)
	{
		int dataLen = 0;
		char* data;
		float r;
		int magicCode;

		if (!readBuffer(&buf, bufSize, &r, sizeof(int)))
			return 11;
		m_instances[i].m_r = r;

		if (!readBuffer(&buf, bufSize, &magicCode, sizeof(int)))
			return 12;
		if (magicCode != MAGIC_CODE_NAVJUMP)
		{
			return 13;
		}
		if (!readBuffer(&buf, bufSize, &dataLen, sizeof(int)))
			return 14;
		data = new char[dataLen];
		if (!readBuffer(&buf, bufSize, data, dataLen))
		{
			delete[] data;
			return 15;
		}
		m_instances[i].m_navMeshJump = m_builderJump->loadAll(data, dataLen);
		delete[] data;
		if (!readBuffer(&buf, bufSize, &dataLen, sizeof(int)))
			return 16;
		data = new char[dataLen];
		if (!readBuffer(&buf, bufSize, data, dataLen))
		{
			delete[] data;
			return 17;
		}
		m_instances[i].m_navMesh = m_builder->loadAll(data, dataLen);
		delete[] data;

		m_instances[i].m_navQueryJump->init(m_instances[i].m_navMeshJump, 2048);
		m_instances[i].m_navQuery->init(m_instances[i].m_navMesh, 2048);
		m_instances[i].queryExt[0] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].queryExt[1] = 1500.0f;
		m_instances[i].queryExt[2] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].navMeshLoaded();
		m_instances[i].m_builder = m_builder;
	}

	//int dataSize = 0;
	//readBuffer(&buf, bufSize, &dataSize, sizeof(int));
	m_builder->addMeshBegin();
	int numMesh;
	int numV;
	int numT;
	readBuffer(&buf, bufSize, &numMesh, sizeof(int));
	for (int i = 0; i < numMesh; i++)
	{
		readBuffer(&buf, bufSize, &numV, sizeof(int));
		const float* vertices = (float*)buf;
		buf += numV * 3 * sizeof(float);
		readBuffer(&buf, bufSize, &numT, sizeof(int));
		const int* triangles = (int*)buf;
		buf += numT * 3 * sizeof(int);
		m_builder->addMesh(0, vertices, numV, triangles, numT);
	}
	m_builder->addMeshEnd();

	return 0;
}

int RecastInterface::build()
{
	for (int i = 0; i < m_agentTypeNum; i++)
	{
		m_builder->setAgentRadius(m_instances[i].m_r);
		m_builder->setAgentMaxClimb(m_paramStepHeight);
		m_builder->buildAllInGame(m_instances[i].m_navMesh);
		m_builder->setAgentMaxClimb(m_paramStepHeightJump);
		m_builder->buildAllInGame(m_instances[i].m_navMeshJump);

		m_instances[i].m_navQueryJump->init(m_instances[i].m_navMeshJump, 2048);
		m_instances[i].m_navQuery->init(m_instances[i].m_navMesh, 2048);
		m_instances[i].queryExt[0] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].queryExt[1] = 1500.0f;
		m_instances[i].queryExt[2] = m_instances[i].m_navMesh->getAgentRadius();
		m_instances[i].navMeshLoaded();
	}
	m_builder->setAgentMaxClimb(m_paramStepHeight);

	return 0;
}

int RecastInterface::update(float dt)
{
	m_instances[0].m_RecastCrowd->updateTick(dt);

	return 0;
}

int RecastInterface::moveInGame(int agentType, float* s, float* e, bool slide)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].moveInGame(s, e, slide);
}

int RecastInterface::moveJumpServer(int agentType, float* s, float* e)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].moveJumpServer(s, e);
}

int RecastInterface::autoMove(int agentType, float* s, float* e, float* path, int& pathNum)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].autoMove(s, e, path, pathNum);
}
/*
int RecastInterface::raycast(int agentType, float* p1, float* p2, float& t)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].raycast(p1, p2, t);
}
*/
int RecastInterface::findMovePath(int agentType, float* s, float* d, float l, float* path, int& pathNum, bool slide)
{
	return m_instances[agentType].findMovePath(s, d, l, path, pathNum, slide);
}

int RecastInterface::getPosNear(int agentType, float& x, float& y, float& z)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getPosNear(x, y, z);
}

int RecastInterface::getPosFall(int agentType, float& x, float& y, float& z)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getPosFall(x, y, z);
}

int RecastInterface::getPosCeil(int agentType, float x, float& y, float z)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getPosCeil(x, y, z);
}

int RecastInterface::getPosFloor(int agentType, float x, float& y, float z)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getPosFloor(x, y, z);
}

int RecastInterface::getPosObstAdjust(int agentType, float& x, float& y, float& z)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getPosObstAdjust(x, y, z);
}

int RecastInterface::getNormal(int agentType, float x, float& y, float z, float* n)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].getNormal(x, y, z, n);
}

int RecastInterface::assureInMap(int agentType, float& x, float& y, float& z)
{
	return m_builder->assureInMap(x, y, z);
}

int RecastInterface::addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret, bool apply)
{
	int status = m_builder->addObstacleCylinder(c, r, h, ret);
	if (status == 0)
	{
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMesh;
			}
			status = m_builder->applyObstacle(m_agentTypeNum, navMeshes);
		}
		unsigned int retJump;
		m_builderJump->addObstacleCylinder(c, r, h, retJump);
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMeshJump;
			}
			status = m_builderJump->applyObstacle(m_agentTypeNum, navMeshes);
		}
		ObstacleInfo oi;
		oi.refMove = ret;
		oi.refMoveJump = retJump;
		m_obstacleInfo[ret] = oi;
		return 0;
	}
#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::addObstacleCylinder error");
#endif
	return status;
}

int RecastInterface::addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret, bool apply)
{
	int status = m_builder->addObstacleBox(c, ext, rad, ret);
	if (status ==0)
	{
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMesh;
			}

			status = m_builder->applyObstacle(m_agentTypeNum, navMeshes);
		}
		unsigned int retJump;
		m_builderJump->addObstacleBox(c, ext, rad, retJump);
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMeshJump;
			}
			status = m_builderJump->applyObstacle(m_agentTypeNum, navMeshes);
		}
		ObstacleInfo oi;
		oi.refMove = ret;
		oi.refMoveJump = retJump;
		m_obstacleInfo[ret] = oi;
		return 0;
	}
#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::addObstacleBox error");
#endif
	return status;
}

int RecastInterface::removeObstacle(unsigned int ref, bool apply)
{
	std::map<unsigned int, ObstacleInfo>::iterator it = m_obstacleInfo.find(ref);
	if (it == m_obstacleInfo.end())
	{
		return 1;
	}
	ObstacleInfo oi = it->second;
	int status = m_builder->removeObstacle(oi.refMove);
	if (status == 0)
	{
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMesh;
			}

			m_builder->applyObstacle(m_agentTypeNum, navMeshes);
		}
		m_builderJump->removeObstacle(oi.refMoveJump);
		if (apply)
		{
			dtNavMesh** navMeshes = new dtNavMesh*[m_agentTypeNum];
			for (int i = 0; i < m_agentTypeNum; i++)
			{
				navMeshes[i] = m_instances[i].m_navMeshJump;
			}
			status = m_builderJump->applyObstacle(m_agentTypeNum, navMeshes);
		}
		return 0;
	}
#ifdef _FREE_SERVER_DEFINE
	ERR_LOG("NavMeshInstance::removeObstacle error");
#endif
	return status;
}

RecastAgent* RecastInterface::getAgent(int index)
{
	assert(index >= 0 && index < m_maxAgents);
	return m_agents + index;
}

int RecastInterface::addAgentIndex(int agentType, bool isPlayer, float* pos, float speed)
{
	assert(agentType >= 0 && agentType < m_agentTypeNum);
	return m_instances[agentType].m_RecastCrowd->addAgent(pos, speed);
}

RecastAgent* RecastInterface::addAgent(int agentType, bool isPlayer, float* pos, float speed)
{
	int index = addAgentIndex(agentType, isPlayer, pos, speed);
	if (index >= 0 && index < m_maxAgents)
	{
		return m_agents + index;
	}
	else
		return 0;
}

int RecastInterface::setAgentTarget(int agentType, int idx, float* targetPos)
{
	m_instances[agentType].m_RecastCrowd->setMoveTarget(idx, targetPos, false);
	return 0;
}

int RecastInterface::getAgentPos(int agentType, int idx, float* pos)
{
	m_instances[agentType].m_RecastCrowd->getPos(idx, pos);
	return 0;
}