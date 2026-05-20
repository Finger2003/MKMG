#pragma once
#include "Curve.h"

struct InterpolatingCurve : public Curve, public NamedObjectCounter<InterpolatingCurve>
{
	DEFINE_TYPE(Curve, ObjectType::InterpolatingCurve);

	InterpolatingCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	InterpolatingCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);
	void UpdatePolyline(const DxDevice& device) override;

	static constexpr const char* SchemaName = "interpolatedC2";
	const char* GetSchemaType() const override { return SchemaName; }
};

