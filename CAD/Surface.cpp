#include "pch.h"
#include "Surface.h"
#include "DxDevice.h"

std::shared_ptr<Point> Surface::GetPoint(int u, int v) const
{
    return m_controlPoints[GetControlPointIndex(u, v)].lock();
}
//
std::vector<unsigned int> Surface::GenerateLineIndices() const
{
	std::vector<unsigned int> indices;
	indices.reserve(static_cast<size_t>((m_gridPointsU - 1) * m_gridPointsV + (m_gridPointsV - 1) * m_gridPointsU) * 2);

	for (int v = 0; v < m_gridPointsV; v++)
	{
		for (int u = 0; u < m_gridPointsU - 1; u++)
		{
			indices.push_back(GetControlPointIndex(u, v));
			indices.push_back(GetControlPointIndex(u + 1, v));
		}
	}

	for (int u = 0; u < m_gridPointsU; u++)
	{
		for (int v = 0; v < m_gridPointsV - 1; ++v)
		{
			indices.push_back(GetControlPointIndex(u, v));
			indices.push_back(GetControlPointIndex(u, v + 1));
		}
	}

	return indices;
}

unsigned int Surface::GetControlPointIndex(int u, int v) const
{
	return static_cast<unsigned int>(v * m_gridPointsU + u);
}

nlohmann::json Surface::Serialize() const
{
	nlohmann::json j = Base::Serialize();

	j["size"] = uint2{ static_cast<unsigned int>(m_gridPointsU), static_cast<unsigned int>(m_gridPointsV) };
	j["samples"] = uint2{ static_cast<unsigned int>(m_linesPerSegmentU), static_cast<unsigned int>(m_linesPerSegmentV) };
	nlohmann::json cpArray = nlohmann::json::array();
	for (unsigned int v = 0; v < m_gridPointsV; ++v)
	{
		for (unsigned int u = 0; u < m_gridPointsU; ++u)
		{
			unsigned int idx = GetControlPointIndex(u, v);
			if (auto cp = m_controlPoints[idx].lock())
				cpArray.push_back({ {"id", cp->m_id} });
			else
				cpArray.push_back(nlohmann::json::object());
		}
	}
	j["controlPoints"] = cpArray;

	return j;
}