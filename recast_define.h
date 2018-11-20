#ifndef RECAST_DEFINE_H_
#define RECAST_DEFINE_H_

#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourCommon.h"
#include "./fastlz.h"

#define EXPECTED_LAYERS_PER_TILE 4
#define FREE_MAPDATA_VERSION 1
#define TILECACHESET_VERSION 1
#define TILECACHESET_MAGIC ('T' << 24 | 'S' << 16 | 'E' << 8 | 'T')

#define  NAVMESHSET_MAGIC ('M' << 24 | 'S' << 16 | 'E' << 8 | 'T')//'MSET';
#define  NAVMESHSET_VERSION 1

enum NMFileType
{
	NM_SOLO = 1,
	NM_TILED = 10,
	NM_TILED_DROPMESH = 11,
	NM_TILED_DROPMESH_DYNAMIC = 12,
	NM_TEMPOBST = 20,
	NM_TEMPOBST_DROPMESH = 21,
	NM_TEMPOBST_DROPMESH_DYNAMIC = 22,
};

enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
};

enum MoveReason
{
	MR_NORMAL = 1,
	MR_BLOCKED = 2,
	MR_SLIDE = 3,
	MR_DROP_LOW = 5,
	MR_DROP_HIGH = 6,
	MR_JUMP = 7,
	MR_CLIMB = 8,
	MR_TODO = 10,
};


struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

#define  MAX_LAYERS 32

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

extern float areaCosts[DT_MAX_AREAS];

bool readBuffer(const char** buf, int& bufSize, void* dst, int size);

int getTilesNum(class dtNavMesh* navmesh);
int getNMDataInfo(class dtNavMesh* navmesh, int index, int& vNum, int& tNum, int& dNum);
int getNMData(class dtNavMesh* navmesh, int index, int& vNum, float* v[], int& tNum, int* t[]);
int getDropData(class dtNavMesh* navmesh, class dtNavMeshQuery* navQuery, int index, int& vNum, float* v[], int& tNum, int* t[]);

#endif	// RECAST_DEFINE_H_