#pragma once
#include "Surface.h"

struct BSplineSurface : public Surface, public NamedObjectCounter<BSplineSurface>
{
	DEFINE_TYPE(Surface, ObjectType::BSplineSurface);
	//static unsigned int s_nextId;

	//BSplineSurface(int uSeg, int vSeg, SurfaceShape shape);
	BSplineSurface(int uGrid, int vGrid, SurfaceShape shape);
	BSplineSurface(unsigned int id, unsigned int nameIndex, std::string&& name,
		uint2 grid, SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints, uint2 samples);
	const char* GetSchemaType() const override { return "bezierSurfaceC2"; }

	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;
protected:
	std::vector<unsigned int> GeneratePatchIndices() const;
	//unsigned int GetGridPointsU() const override;
	//unsigned int GetGridPointsV() const override;
	unsigned int GetPatchDataIndex(int u, int v) const override;
	unsigned int GetBernsteinPointsU() const;
	unsigned int GetBernsteinPointsV() const;
	unsigned int GetBernsteinIndex(int u, int v) const;

	//unsigned int GetExportPointsU() const override { return segmentsU + 3; }
	//unsigned int GetExportPointsV() const override { return segmentsV + 3; }

	unsigned int GetSegmentsU() const;
	unsigned int GetSegmentsV() const;

	void ConvertPatchToBernstein(int patchU, int patchV, std::vector<VertexPosition>& bernsteinGrid) const;
	MathLib::Vec3f Evaluate1D(MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3, int index) const;
};
