#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;

//unsigned int BezierSurface::s_nextId = 0;

//BezierSurface::BezierSurface(int uGrid, int vGrid, SurfaceShape shape)
//	: Base(uGrid, vGrid, shape, "Surface C0 - " + std::to_string(s_nextId++))
//{}
BezierSurface::BezierSurface(int uGrid, int vGrid, SurfaceShape shape)
	: Base(uGrid, vGrid, shape, "Surface C0 - " + std::to_string(s_nextId++))
{}

BezierSurface::BezierSurface(unsigned int id, unsigned int nameIndex, std::string && name, 
	uint2 grid, SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints, uint2 samples)
	: Base(id, std::move(name), grid.u, grid.v, samples.u, samples.v, shape, std::move(controlPoints))
{
	AdvanceCounter(nameIndex);
}

void BezierSurface::InitGeometry(const DxDevice& device)
{
	if (!m_patchVertexBuffer)
	{
		UINT vertexCount = m_gridPointsU * m_gridPointsV;
		m_patchVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(vertexCount);
	}

	m_polylineVertexBuffer = m_patchVertexBuffer;

	if (!m_patchIndexBuffer)
	{
		std::vector<unsigned int> patchIndices = GeneratePatchIndices();
		m_patchIndexCount = static_cast<UINT>(patchIndices.size());
		m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
	}

	if (!m_polylineIndexBuffer)
	{
		std::vector<unsigned int> lineIndices = GenerateLineIndices();
		m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
		m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);
	}
}

void BezierSurface::UpdateVertices(const DxDevice & device)
{
	if (!m_isDirty)
		return;

	//unsigned int pointsU = GetGridPointsU();
	//unsigned int pointsV = GetGridPointsV();
	//UINT vertexCount = pointsU * pointsV;
	UINT vertexCount = m_gridPointsU * m_gridPointsV;

	std::vector<VertexPosition> positions;
	positions.reserve(vertexCount);

	for (unsigned int v = 0; v < m_gridPointsV; v++)
	{
		for (unsigned int u = 0; u < m_gridPointsU; ++u)
		{
			unsigned int idx = GetControlPointIndex(u, v);

			if (auto pt = m_controlPoints[idx].lock())
				positions.push_back({ pt->m_position.x, pt->m_position.y, pt->m_position.z });
			else
				positions.push_back({ 0.0f, 0.0f, 0.0f });
		}
	}

	device.UpdateBuffer(m_polylineVertexBuffer, positions.data(), static_cast<UINT>(positions.size()) * sizeof(VertexPosition));

	m_isDirty = false;
}

unsigned int BezierSurface::GetSegmentsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? m_gridPointsU / 3 : (m_gridPointsU - 1) / 3;
}

unsigned int BezierSurface::GetSegmentsV() const
{
	return (m_gridPointsV - 1) / 3;
}

std::vector<unsigned int> BezierSurface::GeneratePatchIndices() const
{
	std::vector<unsigned int> indices;
	unsigned int segU = GetSegmentsU();
	unsigned int segV = GetSegmentsV();
	indices.reserve(static_cast<size_t>(segU) * segV * 16);

	for (unsigned int patchV = 0; patchV < segV; patchV++)
	{
		for (unsigned int patchU = 0; patchU < segU; patchU++)
		{
			for (int v = 0; v < 4; v++)
			{
				for (int u = 0; u < 4; u++)
				{
					indices.push_back(GetControlPointIndex(patchU * 3 + u, patchV * 3 + v));
				}
			}
		}
	}
	return indices;
}

//unsigned int BezierSurface::GetGridPointsU() const
//{
//	return (shapeType == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);
//}
//
//unsigned int BezierSurface::GetGridPointsV() const
//{
//	return 3 * segmentsV + 1;
//}

unsigned int BezierSurface::GetPatchDataIndex(int u, int v) const
{
	return GetControlPointIndex(u, v);
}