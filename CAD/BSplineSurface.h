#pragma once
#include "Surface.h"

struct BSplineSurface : public Surface, public NamedObjectCounter<BSplineSurface>
{
	DEFINE_TYPE(Surface, ObjectType::BSplineSurface);
	DEFINE_SCHEMA("bezierSurfaceC2");

	BSplineSurface(int uGrid, int vGrid, SurfaceShape shape);
	BSplineSurface(unsigned int id, uint2 grid, uint2 samples, SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);

	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;
protected:
	std::vector<unsigned int> GeneratePatchIndices() const;
	unsigned int GetPatchDataIndex(int u, int v) const override;
	unsigned int GetBernsteinPointsU() const;
	unsigned int GetBernsteinPointsV() const;
	unsigned int GetBernsteinIndex(int u, int v) const;
	unsigned int GetSegmentsU() const;
	unsigned int GetSegmentsV() const;

	void ConvertPatchToBernstein(int patchU, int patchV, std::vector<VertexPosition>& bernsteinGrid) const;
	MathLib::Vec3f Evaluate1D(MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3, int index) const;
};
