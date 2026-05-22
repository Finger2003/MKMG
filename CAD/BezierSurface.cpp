#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;


BezierSurface::BezierSurface(int uGrid, int vGrid, int linesPerSegmentU, int linesPerSegmentV, std::vector<std::weak_ptr<Point>> controlPoints)
	: Base(uGrid, vGrid, linesPerSegmentU, linesPerSegmentV, std::move(controlPoints), "Surface C0 - " + std::to_string(s_nextId++))
{}

BezierSurface::BezierSurface(unsigned int id, uint2 grid, uint2 samples, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData)
	: Base(id, nameData ? std::move(nameData->name) : "Surface C0 - " + std::to_string(s_nextId++), grid.u, grid.v, samples.u, samples.v, std::move(controlPoints))
{
	if (nameData)
		AdvanceCounter(nameData->index);
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

void BezierSurface::InitGeometry(const DxDevice& device, const PrecalculatedSurfaceData& precalculatedData)
{
	m_patchVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.controlPoints);
	m_polylineVertexBuffer = m_patchVertexBuffer;

	m_patchIndexCount = static_cast<UINT>(precalculatedData.patchIndices.size());
	m_patchIndexBuffer = device.CreateIndexBuffer(precalculatedData.patchIndices);

	std::vector<unsigned int> lineIndices = GenerateLineIndices();
	m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);
}

void BezierSurface::UpdateVertices(const DxDevice& device)
{
	if (!m_isDirty)
		return;

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
	return (m_gridPointsU - 1) / 3;
	//return (shapeType == SurfaceShape::Cylinder) ? m_gridPointsU / 3 : (m_gridPointsU - 1) / 3;
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


unsigned int BezierSurface::GetPatchDataIndex(int u, int v) const
{
	return GetControlPointIndex(u, v);
}