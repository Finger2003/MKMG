#include "pch.h"
#include "BSplineCurve.h"
#include "DxDevice.h"
#include "../MathLib/Vec3f.h"

using namespace std;
using namespace MathLib;


BSplineCurve::BSplineCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: Base("BSplineCurve" + to_string(s_nextId++), ObjectType::BSplineCurve, std::move(controlPoints))
{}

BSplineCurve::BSplineCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData)
	: Base(id, nameData ? std::move(nameData->name) : "BSplineCurve" + to_string(s_nextId++), ObjectType::BSplineCurve, std::move(controlPoints))
{
	if (nameData)
		AdvanceCounter(nameData->index);
}


void BSplineCurve::UpdatePolyline(const DxDevice& device)
{
	CleanExpiredPoints();
	if (!m_isDirty)
		return;

	if (m_controlPoints.size() < 2)
	{
		m_lineVertexCount = 0;
		m_curveVertexCount = 0;
		m_bernsteinVertexCount = 0;
		m_isDirty = false;
		return;
	}

	std::vector<Vec3f> currentPositions;
	currentPositions.reserve(m_controlPoints.size());
	for (const auto& wp : m_controlPoints)
	{
		if (auto sp = wp.lock())
			currentPositions.push_back(sp->m_position.ToVec3f());
	}
	size_t n = currentPositions.size();

	// 1. Control Polygon (Straight lines between De Boor points)
	std::vector<VertexPosition> lineVertices;
	if (n > 2)
	{
		lineVertices.reserve(n);
		for (size_t i = 0; i < n; i++)
			lineVertices.push_back({ currentPositions[i].x, currentPositions[i].y, currentPositions[i].z });
	}
	// 2. Convert De Boor to Bernstein for the GS Pipeline
	std::vector<VertexPosition> vertices;
	std::vector<VertexPosition> bernsteinPts;

	std::vector<Vec3f> augPoints;
	augPoints.reserve(n + 2);
	Vec3f pFirst = currentPositions[0];
	Vec3f pSecond = currentPositions[1];
	augPoints.push_back(pFirst * 2.0f - pSecond);
	for (size_t i = 0; i < n; ++i)
		augPoints.push_back(currentPositions[i]);
	Vec3f pLast = currentPositions[n - 1];
	Vec3f pPrev = currentPositions[n - 2];
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

	m_isDirty = false;
}