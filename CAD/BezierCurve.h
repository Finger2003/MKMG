#pragma once
#include "Curve.h"
#include "Point.h"

struct BezierCurve : public Curve, public NamedObjectCounter<BezierCurve>
{
	DEFINE_TYPE(Curve, ObjectType::BezierCurve);

	BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	BezierCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);
	void UpdatePolyline(const DxDevice& device) override;
	static constexpr const char* SchemaName = "bezierC0";
	const char* GetSchemaType() const override { return SchemaName; }
};

