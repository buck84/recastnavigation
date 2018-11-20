#ifndef RECAST_BUILD_H_
#define RECAST_BUILD_H_
//#include "AAssist.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "SampleInterfaces.h"
#include "recast_define.h"
#include "RecastDump.h"
#include <string>
#include <vector>

// BUILD模式遍历mesh即可，不需要考虑效率
// LOAD模式需要缓存每个tile用到的mesh
enum BUILD_MODE
{
	BM_BUILD,
	BM_LOAD
};

struct MeshInfo
{
	// m_meshes中的位置
	int index;
	// 每个mesh用二叉树表示，cids记录当前用到的三角形节点，rcGetChunksOverlappingRect返回的结果
	std::vector<int> cids;
};

// 生成一个tile用到的mesh信息
class tileMeshInfo
{
public:
	tileMeshInfo() : inited(false) {}
	std::vector<MeshInfo> meshes;
	bool inited;
};

// 仿照recast，表示一个障碍
struct frObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
	};

	int touched[DT_MAX_TOUCHED_TILES];
	unsigned short salt;
	unsigned char type;
	unsigned char state;
	unsigned char ntouched;
	class rcMeshLoaderObj* mesh;
	frObstacle* next;
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};

class RaycastBuild
{
public:
	RaycastBuild();
	~RaycastBuild() {}

	int addMeshBegin();
	int addMesh(int areaType, const float* vertices, const int vCount, const int* triangles, const int tCount);
	virtual int addMeshEnd();
	int exportMesh(std::string name);
	virtual int build(bool genDrop) = 0;
	virtual int buildAllInGame(dtNavMesh*& navmeshBuild) { return 0;  }
	virtual dtNavMesh* loadAll(const char* buf, int bufSize) = 0;

	class dtNavMesh* getNavMesh() { return m_navMesh; }
	class dtNavMeshQuery* getNavQuery() { return m_navQuery; }

	void setCellSize(float cs) { m_cellSize = cs; }
	void setCellHeight(float ch) { m_cellHeight = ch; }
	void setTileSize(float ts) { m_tileSize = ts; }

	void setAgentRadius(float ar) { m_agentRadius = ar; }
	void setAgentHeight(float ah) { m_agentHeight = ah; }
	void setAgentMaxSlope(float ms) { m_agentMaxSlope = ms; }
	void setAgentMaxClimb(float mc) { m_agentMaxClimb = mc; }
	void setDropHeightAutoMove(float dh) { m_dropHeightAutoMove = dh; }
	void setDropHoriDistScale(float dh) { m_dropHoriDistScale = dh; }
	
	virtual int saveAll(const char* path) { return 0;  }

	void clearOffmeshJumpedge();
	void addOffMeshConnection(const float* spos, const float* epos, const float rad,
		unsigned char bidir, unsigned char area, unsigned short flags);
	int deleteOffMeshConnection(const float* pos);
	
	int getOffMeshCount();
	int getOffMeshData(float* p[]);

	virtual int assureInMap(float& x, float& y, float& z) = 0;

	virtual int addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret);
	virtual int addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret);
	virtual int removeObstacle(unsigned int ref);
	virtual dtStatus applyObstacle(int numNav, class dtNavMesh** navmeshes, int maxTile = 10);

	bool blockPlayer(float* s, float* e);

protected:
	class InputGeom* m_geom;
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_dropHeight = 99.0f;
	float m_dropHeightAutoMove = 5.0f;
	float m_dropHoriDistScale = 3.0f;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;
	float m_tileSize;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;

	BuildContext* m_ctx;

	rcHeightfield** m_solidCache;
	int m_tileWidth;
	int m_tileHeight;
	int m_widthTile;
	int m_heightTile;

	float m_BMin[3];
	float m_BMax[3];
	
	void resetCommonSettings();
};

#endif

