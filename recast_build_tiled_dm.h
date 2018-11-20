#ifndef RECAST_BUILD_TILED_DROPMESH_H_
#define RECAST_BUILD_TILED_DROPMESH_H_

#include "recast_build.h"

class RaycastBuildTiledDropMesh : public RaycastBuild
{
public:
	RaycastBuildTiledDropMesh();
	~RaycastBuildTiledDropMesh();

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
	unsigned char* m_triareas;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;
	rcPolyMeshDetail* m_dmesh;
	rcConfig m_cfg;

	int m_maxTiles;
	int m_maxPolysPerTile;

	float m_tileBuildTime;
	float m_tileMemUsage;
	int m_tileTriCount;

	tileMeshInfo *m_tileMeshInfos;

	BUILD_MODE m_buildMode;

	float m_jumpDistHori;

protected:
	void cleanup();
	void buildAllTiles();
	unsigned char* buildTileMesh(const int tx, const int ty, const float* bmin, const float* bmax, int& dataSize);
	unsigned char* buildTileDropMesh(int index, int& dataSize, class dtNavMesh* navmesh);

	bool dropNoObstacleHori(float p[4][3], class dtNavMesh* navmesh);
	bool insectSolid(float* ps, float* pd, int x, int y);

	virtual dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);
	dtStatus genHeightField(const int tx, const int ty);

	frObstacle* m_obstacles;
	frObstacle* m_nextFreeObstacle;
	frObstacle* m_obstaclesInUse;
	int m_maxObstacles;

	enum ObstacleRequestAction
	{
		REQUEST_ADD,
		REQUEST_REMOVE,
	};

	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};
	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_reqs[MAX_REQUESTS];
	int m_nreqs;

	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;


	/// Encodes an obstacle id.
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}

	/// Decodes an obstacle salt.
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1 << 16) - 1;
		return (unsigned int)((ref >> 16) & saltMask);
	}

	/// Decodes an obstacle id.
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1 << 16) - 1;
		return (unsigned int)(ref & tileMask);
	}
	inline const frObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }

	frObstacle* getObstacleByRef(dtObstacleRef ref);

	dtObstacleRef getObstacleRef(const frObstacle* obmin) const;
	void getObstacleBounds(const struct frObstacle* ob, float* bmin, float* bmax) const;
};

#endif // RECAST_BUILD_TILED_DROPMESH_H_

