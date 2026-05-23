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

BezierCurve::BezierCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData)
	: Base(id, nameData ? std::move(nameData->name) : "BezierCurve" + to_string(s_nextId++), ObjectType::BezierCurve, std::move(controlPoints))
{
	if (nameData)
		AdvanceCounter(nameData->index);
}

void BezierCurve::UpdatePolyline(const DxDevice& device)
{
	CleanExpiredPoints();
	if (!m_isDirty)
		return;

	if (m_controlPoints.size() < 2)
	{
		m_lineVertexCount = 0;
		m_curveVertexCount = 0;
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

	size_t polylinePoints = (n % 3 == 2) ? (n - 1) : n;
	std::vector<VertexPosition> lineVertices;
	lineVertices.reserve(polylinePoints);
	for (size_t i = 0; i < polylinePoints; i++)
		lineVertices.push_back({ currentPositions[i].x, currentPositions[i].y, currentPositions[i].z });

	std::vector<VertexPosition> vertices;
	vertices.reserve(n + (n / 3) * 3);
	for (size_t i = 0; i + 1 < n; i += 3)
	{
		Vec3f p0 = currentPositions[i];
		Vec3f p1, p2, p3;
		if (i + 3 < n)
		{
			p1 = currentPositions[i + 1];
			p2 = currentPositions[i + 2];
			p3 = currentPositions[i + 3];
		}
		else if (i + 2 < n)
		{
			// p0=b0, b1, p3=b2
			Vec3f b1 = currentPositions[i + 1];
			p3 = currentPositions[i + 2];
			p1 = (b1 - p0) * (2.0f / 3.0f) + p0;
			p2 = (b1 - p3) * (2.0f / 3.0f) + p3;
		}
		else
		{
			// p0=b0, p3=b1
			p3 = currentPositions[i + 1];
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

	m_isDirty = false;
}