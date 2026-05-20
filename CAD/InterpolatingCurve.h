#pragma once
#include "Curve.h"

struct InterpolatingCurve : public Curve, public NamedObjectCounter<InterpolatingCurve>
{
	DEFINE_TYPE(Curve, ObjectType::InterpolatingCurve);
	DEFINE_SCHEMA("interpolatedC2");

	InterpolatingCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	InterpolatingCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);
	void UpdatePolyline(const DxDevice& device) override;


};

