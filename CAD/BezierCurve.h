#pragma once
#include "SceneObject.h"
#include "Point.h"

struct BezierCurve : public SceneObject
{
	static unsigned int s_nextId;
	std::vector<std::weak_ptr<Point>> m_controlPoints;

	BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	void CleanExpiredPoints();
};

