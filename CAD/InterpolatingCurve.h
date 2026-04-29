#pragma once
#include "Curve.h"

struct InterpolatingCurve : public Curve
{
	static unsigned int s_nextId;

	InterpolatingCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	void UpdatePolyline(const DxDevice& device) override;
};

