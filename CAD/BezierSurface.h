#pragma once
#include "SceneObject.h"
#include "Point.h"
#include "Surface.h"



struct BezierSurface;

struct BezierSurface : public Surface
{
	DEFINE_TYPE(Surface, ObjectType::BezierSurface);
	static unsigned int s_nextId;	

	BezierSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview = false);
	void Commit() override;

	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;
	static SurfaceGenerationResult CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device);
	static SurfaceGenerationResult CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device);
protected:
	unsigned int GetGridPointsU() const override;
	unsigned int GetGridPointsV() const override;
	unsigned int GetPatchDataIndex(int u, int v) const override;
};
