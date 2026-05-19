#include "pch.h"
#include "Surface.h"
#include "DxDevice.h"

std::shared_ptr<Point> Surface::GetPoint(int u, int v) const
{
    return m_controlPoints[GetControlPointIndex(u, v)].lock();
}
//
//std::vector<unsigned int> Surface::GenerateLineIndices() const
//{
//	std::vector<unsigned int> indices;
//
//	int pointsU = GetGridPointsU();
//	int pointsV = GetGridPointsV();
//	int logicalPointsU = (shapeType == SurfaceShape::Cylinder) ? pointsU + 1 : pointsU;
//
//	for (int v = 0; v < pointsV; v++)
//	{
//		for (int u = 0; u < logicalPointsU - 1; u++)
//		{
//			indices.push_back(GetControlPointIndex(u, v));
//			indices.push_back(GetControlPointIndex(u + 1, v));
//		}
//	}
//
//	for (int u = 0; u < pointsU; u++)
//	{
//		for (int v = 0; v < pointsV - 1; ++v)
//		{
//			indices.push_back(GetControlPointIndex(u, v));
//			indices.push_back(GetControlPointIndex(u, v + 1));
//		}
//	}
//
//	return indices;
//}
//
//std::vector<unsigned int> Surface::GeneratePatchIndices() const
//{
//	std::vector<unsigned int> indices;
//	indices.reserve(static_cast<size_t>(segmentsU) * segmentsV * 16);
//
//	for (int patchV = 0; patchV < segmentsV; patchV++)
//	{
//		for (int patchU = 0; patchU < segmentsU; patchU++)
//		{
//			for (int v = 0; v < 4; v++)
//			{
//				for (int u = 0; u < 4; u++)
//				{
//					indices.push_back(GetPatchDataIndex(patchU * 3 + u, patchV * 3 + v));
//				}
//			}
//		}
//	}
//	return indices;
//}

unsigned int Surface::GetControlPointIndex(int u, int v) const
{
	unsigned int pointsU = GetGridPointsU();
	int wrappedU = (shapeType == SurfaceShape::Cylinder) ? (u % pointsU) : u;
	return static_cast<unsigned int>(v * pointsU + wrappedU);
}

nlohmann::json Surface::Serialize() const
{
	nlohmann::json j = Base::Serialize();
	unsigned int exportU = GetExportPointsU();
	unsigned int exportV = GetExportPointsV();

	j["size"] = { exportU, exportV };
	j["samples"] = { m_linesPerSegmentU, m_linesPerSegmentV };
	nlohmann::json cpArray = nlohmann::json::array();
	for (unsigned int v = 0; v < exportV; ++v)
	{
		for (unsigned int u = 0; u < exportU; ++u)
		{
			unsigned int idx = GetControlPointIndex(u, v);
			if (auto cp = m_controlPoints[idx].lock())
				cpArray.push_back({ {"id", cp->m_id} });
		}
	}
	j["controlPoints"] = cpArray;
	return j;
}