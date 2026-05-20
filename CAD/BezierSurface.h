#pragma once
#include "SceneObject.h"
#include "Point.h"
#include "Surface.h"



struct BezierSurface;

struct BezierSurface : public Surface, public NamedObjectCounter<BezierSurface>
{
	DEFINE_TYPE(Surface, ObjectType::BezierSurface);
	BezierSurface(int uGrid, int vGrid, SurfaceShape);
	BezierSurface(unsigned int id, uint2 grid, uint2 samples, SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);

	static constexpr const char* SchemaName = "bezierSurfaceC0";
	const char* GetSchemaType() const override { return SchemaName; }

	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;
protected:
	unsigned int GetSegmentsU() const;
	unsigned int GetSegmentsV() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	unsigned int GetPatchDataIndex(int u, int v) const override;
};
