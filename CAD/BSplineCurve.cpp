#include "pch.h"
#include "BSplineCurve.h"
#include "DxDevice.h"
#include "../MathLib/Vec3f.h"

using namespace std;
using namespace MathLib;

//unsigned int BSplineCurve::s_nextId = 0;

BSplineCurve::BSplineCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: Base("BSplineCurve" + to_string(s_nextId++), ObjectType::BSplineCurve, std::move(controlPoints))
{}

BSplineCurve::BSplineCurve(unsigned int id, unsigned int bsplineCurveIndex, std::string&& name, std::vector<std::weak_ptr<Point>>&& controlPoints)
	: Base(id, std::move(name), ObjectType::BSplineCurve, std::move(controlPoints))
{
	AdvanceCounter(bsplineCurveIndex);
}

void BSplineCurve::UpdatePolyline(const DxDevice& device)
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
				m_lastPositions.push_back(sp->m_position);
		}

		size_t n = m_lastPositions.size();

		// 1. Control Polygon (Straight lines between De Boor points)
		std::vector<VertexPosition> lineVertices;
		if (n > 2)
		{
			lineVertices.reserve(n);
			for (size_t i = 0; i < n; i++)
				lineVertices.push_back({ m_lastPositions[i].x, m_lastPositions[i].y, m_lastPositions[i].z });
		}
		// 2. Convert De Boor to Bernstein for the GS Pipeline
		std::vector<VertexPosition> vertices;
		std::vector<VertexPosition> bernsteinPts;

		std::vector<Vec3f> augPoints;
		augPoints.reserve(n + 2);
		Vec3f pFirst = m_lastPositions[0].ToVec3f();
		Vec3f pSecond = m_lastPositions[1].ToVec3f();
		augPoints.push_back(pFirst * 2.0f - pSecond);
		for (size_t i = 0; i < n; ++i)
			augPoints.push_back(m_lastPositions[i].ToVec3f());
		Vec3f pLast = m_lastPositions[n - 1].ToVec3f();
		Vec3f pPrev = m_lastPositions[n - 2].ToVec3f();
		augPoints.push_back(pLast * 2.0f - pPrev);
		size_t augN = augPoints.size();

		vertices.reserve((augN - 3) * 4);
		bernsteinPts.reserve((augN - 3) * 3 + 1);
		m_virtualPoints.clear();
		m_virtualPoints.reserve((augN - 3) * 3 + 1);

		for (size_t i = 0; i <= augN - 4; i++)
		{
			Vec3f p0 = augPoints[i];
			Vec3f p1 = augPoints[i + 1];
			Vec3f p2 = augPoints[i + 2];
			Vec3f p3 = augPoints[i + 3];

			// Convert B-Spline segment to Bezier segment
			Vec3f b0 = (p0 + p1 * 4.0f + p2) * (1.0f / 6.0f);
			Vec3f b1 = (p1 * 2.0f + p2) * (1.0f / 3.0f);
			Vec3f b2 = (p1 + p2 * 2.0f) * (1.0f / 3.0f);
			Vec3f b3 = (p1 + p2 * 4.0f + p3) * (1.0f / 6.0f);

			vertices.push_back({ b0.x, b0.y, b0.z });
			vertices.push_back({ b1.x, b1.y, b1.z });
			vertices.push_back({ b2.x, b2.y, b2.z });
			vertices.push_back({ b3.x, b3.y, b3.z });

			bernsteinPts.push_back({ b0.x, b0.y, b0.z });
			bernsteinPts.push_back({ b1.x, b1.y, b1.z });
			bernsteinPts.push_back({ b2.x, b2.y, b2.z });
			if (i == augN - 4)
				bernsteinPts.push_back({ b3.x, b3.y, b3.z });


			auto& real_p_i = m_controlPoints[i];
			auto& real_p_i_plus_1 = m_controlPoints[i + 1];
			m_virtualPoints.push_back({ real_p_i, (i == 0) ? 1.0f : (2.0f / 3.0f), b0 });
			m_virtualPoints.push_back({ real_p_i, 2.0f / 3.0f, b1 });
			m_virtualPoints.push_back({ real_p_i_plus_1, 2.0f / 3.0f, b2 });
			if (i == augN - 4)
				m_virtualPoints.push_back({ real_p_i_plus_1, 1.0f, b3 });
		}

		UpdateBuffer(device, m_lineVertexBuffer, m_lineBufferCapacity, lineVertices, m_lineVertexCount, 2);
		UpdateBuffer(device, m_curveVertexBuffer, m_curveBufferCapacity, vertices, m_curveVertexCount, 4);
		UpdateBuffer(device, m_bernsteinVertexBuffer, m_bernsteinBufferCapacity, bernsteinPts, m_bernsteinVertexCount, 2);
	}
}