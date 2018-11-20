#ifndef RECAST_BUILD_TEMPOBSTACLE_H_
#define RECAST_BUILD_TEMPOBSTACLE_H_

#include "recast_build.h"
#include <vector>


class RaycastBuildTempObstacle : public RaycastBuild
{
public:
	RaycastBuildTempObstacle();
	~RaycastBuildTempObstacle();

	// mesh生成结束，根据mesh信息初始化各种参数
	virtual int addMeshEnd();
	int build(bool genDrop);

	int saveAll(const char* path);
	dtNavMesh* loadAll(const char* buf, int bufSize);

	int addObstacleCylinder(const float* c, const float r, const float h, unsigned int& ret);
	int addObstacleBox(const float* c, const float* ext, const float rad, unsigned int& ret);
	int removeObstacle(unsigned int ref);
	dtStatus applyObstacle(int numNav, class dtNavMesh** navmeshes, int maxTile = 10);

	int assureInMap(float& x, float& y, float& z);
private:

	bool m_keepInterResults;

	int m_maxTiles;
	int m_maxPolysPerTile;

	float m_tileBuildTime;
	float m_tileMemUsage;
	int m_tileTriCount;

	float m_jumpDistHori;

	struct LinearAllocator* m_talloc;
	struct FastLZCompressor* m_tcomp;
	struct MeshProcess* m_tmproc;

	class dtTileCache* m_tileCache;

	float m_cacheBuildTimeMs;
	int m_cacheCompressedSize;
	int m_cacheRawSize;
	int m_cacheLayerCount;
	int m_cacheBuildMemUsage;
	float m_tileSize;
	int m_maxTrisPerChunk;

protected:
	void cleanup();
	int rasterizeTileLayers(bool genDrop, const int tx, const int ty, const rcConfig& cfg, struct TileCacheData* tiles, const int maxTiles);

};

#endif // RECAST_BUILD_TEMPOBSTACLE_H_

