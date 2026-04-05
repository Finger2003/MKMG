#include "pch.h"
#include "BezierCurve.h"

using namespace std;

unsigned int BezierCurve::s_nextId = 0;

BezierCurve::BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: SceneObject("BezierCurve" + to_string(s_nextId++), ObjectType::BezierCurve), m_controlPoints(std::move(controlPoints))
{}

void BezierCurve::CleanExpiredPoints()
{
	erase_if(m_controlPoints, [](const weak_ptr<Point>& wp) { return wp.expired(); });
}