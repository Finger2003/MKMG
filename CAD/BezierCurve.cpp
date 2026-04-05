#include "pch.h"
#include "BezierCurve.h"
#include "DxDevice.h"

using namespace std;

unsigned int BezierCurve::s_nextId = 0;

BezierCurve::BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: SceneObject("BezierCurve" + to_string(s_nextId++), ObjectType::BezierCurve), m_controlPoints(std::move(controlPoints))
{}

void BezierCurve::CleanExpiredPoints()
{
	erase_if(m_controlPoints, [](const weak_ptr<Point>& wp) { return wp.expired(); });
}

void BezierCurve::UpdatePolyline(const DxDevice& device)
{
	CleanExpiredPoints();

	if (m_controlPoints.size() < 2)
	{
		m_vertexBuffer.Reset();
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
		std::vector<VertexPosition> vertices;
		for (const auto& wp : m_controlPoints)
		{
			if (auto sp = wp.lock())
			{
				m_lastPositions.push_back(sp->m_position);
				vertices.push_back({ sp->m_position.x, sp->m_position.y, sp->m_position.z });
			}
		}

		if (vertices.size() >= 2)
			m_vertexBuffer = device.CreateVertexBuffer(vertices);
		else 
			m_vertexBuffer.Reset();
	}
}
