#pragma once
#include "Curve.h"
#include "Point.h"

struct BezierCurve : public Curve, public NamedObjectCounter<BezierCurve>
{
	DEFINE_TYPE(Curve, ObjectType::BezierCurve);
	//static unsigned int s_nextId;

	BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	BezierCurve(unsigned int id, unsigned int bezierCurveIndex, std::string&& name, std::vector<std::weak_ptr<Point>>&& controlPoints);
	void UpdatePolyline(const DxDevice& device) override;
	const char* GetSchemaType() const override { return "bezierC0"; }
};

