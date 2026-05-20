#include "pch.h"
#include "BezierCurve.h"
#include "DxDevice.h"
#include "../MathLib/Vec3f.h"

using namespace std;
using namespace MathLib;

//unsigned int BezierCurve::s_nextId = 0;

BezierCurve::BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: Base("BezierCurve" + to_string(s_nextId++), ObjectType::BezierCurve, std::move(controlPoints))
{}

BezierCurve::BezierCurve(unsigned int id, unsigned int bezierCurveIndex, std::string&& name, std::vector<std::weak_ptr<Point>> && controlPoints)
	: Base(id, std::move(name), ObjectType::BezierCurve, std::move(controlPoints))
{
	AdvanceCounter(bezierCurveIndex);
}

//void BezierCurve::CleanExpiredPoints()
//{
//	erase_if(m_controlPoints, [](const weak_ptr<Point>& wp) { return wp.expired(); });
//}

void BezierCurve::UpdatePolyline(const DxDevice& device)
{
	CleanExpiredPoints();

	if (m_controlPoints.size() < 2)
	{
		m_lineVertexCount = 0;
		m_curveVertexCount = 0;
		return;
	}

	bool needUpdate = false;
	if (m_lastPositions.size() != m_controlPoints.size())
		needUpdate = true;
	else
	{
		for (size_t i = 0; i < m_controlPoints.size(); ++i)
		{
			auto sp = m_controlPoints[i].lock();
			if (!sp ||
				sp->m_position.x != m_lastPositions[i].x ||
				sp->m_position.y != m_lastPositions[i].y ||
				sp->m_position.z != m_lastPositions[i].z)
			{
				needUpdate = true;
				break;
			}
		}
	}

	if (needUpdate)
	{
		m_lastPositions.clear();
		for (const auto& wp : m_controlPoints)
		{
			if (auto sp = wp.lock())
			{
				m_lastPositions.push_back(sp->m_position);				
			}
		}

		size_t n = m_lastPositions.size();

		size_t polylinePoints = (n % 3 == 2) ? (n - 1) : n;
		std::vector<VertexPosition> lineVertices;
		lineVertices.reserve(polylinePoints);
		for (size_t i = 0; i < polylinePoints; i++)
			lineVertices.push_back({ m_lastPositions[i].x, m_lastPositions[i].y, m_lastPositions[i].z });

		std::vector<VertexPosition> vertices;
		vertices.reserve(n + (n / 3) * 3);
		for (size_t i = 0; i + 1 < n; i += 3)
		{
			Vec3f p0 = m_lastPositions[i].ToVec3f();
			Vec3f p1, p2, p3;
			if (i + 3 < n)
			{
				p1 = m_lastPositions[i + 1].ToVec3f();
				p2 = m_lastPositions[i + 2].ToVec3f();
				p3 = m_lastPositions[i + 3].ToVec3f();
			}
			else if (i + 2 < n)
			{
				// p0=b0, b1, p3=b2
				Vec3f b1 = m_lastPositions[i + 1].ToVec3f();
				p3 = m_lastPositions[i + 2].ToVec3f();
				p1 = (b1 - p0) * (2.0f / 3.0f) + p0;
				p2 = (b1 - p3) * (2.0f / 3.0f) + p3;
			}
			else
			{
				// p0=b0, p3=b1
				p3 = m_lastPositions[i + 1].ToVec3f();
				p1 = (p3 - p0) * (1.0f / 3.0f) + p0;
				p2 = (p3 - p0) * (2.0f / 3.0f) + p0;
			}

			vertices.push_back({ p0.x, p0.y, p0.z });
			vertices.push_back({ p1.x, p1.y, p1.z });
			vertices.push_back({ p2.x, p2.y, p2.z });
			vertices.push_back({ p3.x, p3.y, p3.z });
		}

		UpdateBuffer(device, m_lineVertexBuffer, m_lineBufferCapacity, lineVertices, m_lineVertexCount, 2);
		UpdateBuffer(device, m_curveVertexBuffer, m_curveBufferCapacity, vertices, m_curveVertexCount, 4);
	}
}