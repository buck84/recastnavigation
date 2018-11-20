#include "recast_define.h"
#include <cstring>

float areaCosts[DT_MAX_AREAS];


bool readBuffer(const char** buf, int& bufSize, void* dst, int size)
{
	if (bufSize < size)
	{
		return false;
	}
	memcpy(dst, *buf, size);
	*buf += size;
	bufSize -= size;
	return true;
}

int getTilesNum(class dtNavMesh* navmesh)
{
	if (!navmesh)
	{
		return 0;
	}
	const dtNavMesh& mesh = *navmesh;
	for (int i = navmesh->getMaxTiles() - 1; i >= 0; --i)
	{
		const dtMeshTile* tile = mesh.getTile(i);
		if (tile->header)
			return i + 1;
	}
	return 0;

}
int getNMDataInfo(class dtNavMesh* navmesh, int index, int& vNum, int& tNum, int& dNum)
{
	vNum = 0;
	tNum = 0;
	dNum = 0;
	const dtNavMesh& mesh = *navmesh;
	const dtMeshTile* tile = mesh.getTile(index);
	if (!tile->header)
		return 0;

	vNum += (tile->header->vertCount + tile->header->detailVertCount);

	int vCount = tile->header->vertCount;

	for (int i = 0; i < tile->header->polyCount + tile->header->dropEdgeCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_GROUND)
		{
			const dtPolyDetail* pd = &tile->detailMeshes[i];
			tNum += pd->triCount;
		}
		else if (p->getType() == DT_POLYTYPE_OFFMESH_DROP)
		{
			dNum++;
		}
	}

	return 0;
}

int getNMData(class dtNavMesh* navmesh, int index, int& vNum, float* v[], int& tNum, int* t[])
{
	int& mOutNumVert = vNum;
	float*& mOutVertices = *v;
	int& mOutNumTri = tNum;
	int*& mOutTriangles = *t;

	const dtNavMesh& mesh = *navmesh;
	mOutNumVert = 0;
	mOutNumTri = 0;
	const dtMeshTile* tile = mesh.getTile(index);
	if (!tile->header)
		return 0;

	mOutNumVert += (tile->header->vertCount + tile->header->detailVertCount);

	//mOutVertices = new float[mOutNumVert * 3];

	int vCount = tile->header->vertCount;
	for (int j = 0; j < tile->header->vertCount; j++)
	{
		const float x = tile->verts[j * 3];
		const float y = tile->verts[j * 3 + 1];
		const float z = tile->verts[j * 3 + 2];
		mOutVertices[j * 3 + 0] = x;
		mOutVertices[j * 3 + 1] = y;
		mOutVertices[j * 3 + 2] = z;
	}

	for (int j = 0; j < tile->header->detailVertCount; j++)
	{
		const float x = tile->detailVerts[j * 3];
		const float y = tile->detailVerts[j * 3 + 1];
		const float z = tile->detailVerts[j * 3 + 2];
		mOutVertices[(vCount + j) * 3 + 0] = x;
		mOutVertices[(vCount + j) * 3 + 1] = y;
		mOutVertices[(vCount + j) * 3 + 2] = z;
	}

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;

		const dtPolyDetail* pd = &tile->detailMeshes[i];
		mOutNumTri += pd->triCount;
	}

	//mOutTriangles = new int[mOutNumTri*3];
	int tIndex = 0;

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
			continue;

		const dtPolyDetail* pd = &tile->detailMeshes[i];
		for (int j = 0; j < pd->triCount; ++j)
		{
			int tri[3];
			const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->vertCount)
					tri[k] = p->verts[t[k]];
				//dd->vertex(&tile->verts[p->verts[t[k]] * 3], col);
				else
					tri[k] = vCount + (pd->vertBase + t[k] - p->vertCount);
				//dd->vertex(&tile->detailVerts[(pd->vertBase + t[k] - p->vertCount) * 3], col);
			}
			mOutTriangles[tIndex++] = tri[0];
			mOutTriangles[tIndex++] = tri[1];
			mOutTriangles[tIndex++] = tri[2];
		}
	}

	//memcpy(*v, mOutVertices, mOutNumVert * 3 * sizeof(float));
	//memcpy(*t, mOutTriangles, mOutNumTri * 3 * sizeof(int));
	//vNum = mOutNumVert;
	//tNum = mOutNumTri;
	return 0;
}

int getDropData(class dtNavMesh* navmesh, class dtNavMeshQuery* navQuery, int index, int& vNum, float* v[], int& tNum, int* t[])
{
	vNum = 0;
	tNum = 0;
	const dtNavMesh& mesh = *navmesh;
	const dtMeshTile* tile = mesh.getTile(index);
	if (!tile->header)
		return 0;
	float vert1[3], vert2[3], vert3[3], vert4[4];
	dtPolyRef base = navmesh->getPolyRefBase(tile);
	int dropNum = 0;
	const float s = 1.0f / 255.0f;
	for (int i1 = tile->header->polyCount; i1 < tile->header->polyCount + tile->header->dropEdgeCount; i1++)
	{
		const dtPoly* p = &tile->polys[i1];
		if (p->getType() != DT_POLYTYPE_OFFMESH_DROP)
			continue;
		float* v0 = &tile->verts[p->verts[0] * 3];
		float* v1 = &tile->verts[p->verts[1] * 3];
		float v2[3], v3[3];
		int smin = p->neis[0] + 1;
		int smax = p->neis[1] - 1;
		dtVlerp(v2, v0, v1, s*smin);
		dtVlerp(v3, v0, v1, s*smax);

		dtStatus ret = navQuery->getLandPos(v2, vert3, base | i1, vert1);
		if (ret != DT_SUCCESS)
		{
			dtVcopy(vert3, vert1);
		}
		ret = navQuery->getLandPos(v3, vert4, base | i1, vert2);
		if (ret != DT_SUCCESS)
		{
			dtVcopy(vert4, vert2);
		}
		dtVcopy((*v) + vNum * 3, v2);
		vNum++;
		dtVcopy((*v) + vNum * 3, v3);
		vNum++;
		dtVcopy((*v) + vNum * 3, vert1);
		vNum++;
		dtVcopy((*v) + vNum * 3, vert2);
		vNum++;
		dtVcopy((*v) + vNum * 3, vert3);
		vNum++;
		dtVcopy((*v) + vNum * 3, vert4);
		vNum++;
		(*t)[tNum * 3] = 6 * dropNum + 0;
		(*t)[tNum * 3 + 1] = 6 * dropNum + 3;
		(*t)[tNum * 3 + 2] = 6 * dropNum + 1;
		tNum++;
		(*t)[tNum * 3] = 6 * dropNum + 0;
		(*t)[tNum * 3 + 1] = 6 * dropNum + 2;
		(*t)[tNum * 3 + 2] = 6 * dropNum + 3;
		tNum++;
		(*t)[tNum * 3] = 6 * dropNum + 2;
		(*t)[tNum * 3 + 1] = 6 * dropNum + 5;
		(*t)[tNum * 3 + 2] = 6 * dropNum + 3;
		tNum++;
		(*t)[tNum * 3] = 6 * dropNum + 2;
		(*t)[tNum * 3 + 1] = 6 * dropNum + 4;
		(*t)[tNum * 3 + 2] = 6 * dropNum + 5;
		tNum++;
		dropNum++;
	}
	return 0;
}