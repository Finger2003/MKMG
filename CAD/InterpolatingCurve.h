#pragma once
#include "Curve.h"

struct InterpolatingCurve : public Curve, public NamedObjectCounter<InterpolatingCurve>
{
	DEFINE_TYPE(Curve, ObjectType::InterpolatingCurve);
	//static unsigned int s_nextId;

	InterpolatingCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	InterpolatingCurve(unsigned int id, unsigned int interpolatingCurveIndex, std::string&& name, std::vector<std::weak_ptr<Point>>&& controlPoints);
	void UpdatePolyline(const DxDevice& device) override;

	const char* GetSchemaType() const override { return "interpolatedC2"; }
};

