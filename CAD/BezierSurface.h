#pragma once
#include "SceneObject.h"
#include "Point.h"
#include "Surface.h"



struct BezierSurface;

struct BezierSurface : public Surface, public NamedObjectCounter<BezierSurface>
{
	DEFINE_TYPE(Surface, ObjectType::BezierSurface);
	DEFINE_SCHEMA("bezierSurfaceC0");

	BezierSurface(int uGrid, int vGrid, int linesPerSegmentU, int linesPerSegmentV, std::vector<std::weak_ptr<Point>> controlPoints);
	//BezierSurface(int uGrid, int vGrid);
	BezierSurface(unsigned int id, uint2 grid, uint2 samples, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);

	void InitGeometry(const DxDevice& device) override;
	void InitGeometry(const DxDevice& device, const PrecalculatedSurfaceData& precalculatedData) override;
	void UpdateVertices(const DxDevice& device) override;
protected:
	unsigned int GetSegmentsU() const;
	unsigned int GetSegmentsV() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	unsigned int GetPatchDataIndex(int u, int v) const override;
};
