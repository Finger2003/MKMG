#pragma once
#include "SceneObject.h"
#include "Point.h"
#include "Surface.h"



struct BezierSurface;

struct BezierSurface : public Surface, public NamedObjectCounter<BezierSurface>
{
	DEFINE_TYPE(Surface, ObjectType::BezierSurface);
	//static unsigned int s_nextId;	

	//BezierSurface(int uSeg, int vSeg, SurfaceShape shape);
	BezierSurface(int uGrid, int vGrid, SurfaceShape);
	BezierSurface(unsigned int id, unsigned int nameIndex, std::string&& name,
		uint2 grid, SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints, uint2 samples);

	const char* GetSchemaType() const override { return "bezierSurfaceC0"; }

	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;
protected:
	unsigned int GetSegmentsU() const;
	unsigned int GetSegmentsV() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	//unsigned int GetGridPointsU() const override;
	//unsigned int GetGridPointsV() const override;
	unsigned int GetPatchDataIndex(int u, int v) const override;
	//unsigned int GetExportPointsU() const override { return 3 * segmentsU + 1; }
	//unsigned int GetExportPointsV() const override { return 3 * segmentsV + 1; }
};
