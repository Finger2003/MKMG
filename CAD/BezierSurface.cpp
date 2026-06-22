#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;


BezierSurface::BezierSurface(int uGrid, int vGrid, int linesPerSegmentU, int linesPerSegmentV, std::vector<std::weak_ptr<Point>> controlPoints)
	: Base(uGrid, vGrid, linesPerSegmentU, linesPerSegmentV, std::move(controlPoints), "Surface C0 - " + std::to_string(s_nextId++), ObjectType::BezierSurface)
{}

BezierSurface::BezierSurface(unsigned int id, uint2 grid, uint2 samples, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData)
	: Base(id, nameData ? std::move(nameData->name) : "Surface C0 - " + std::to_string(s_nextId++), grid.u, grid.v, samples.u, samples.v, std::move(controlPoints), ObjectType::BezierSurface)
{
	if (nameData)
		AdvanceCounter(nameData->index);
}

void BezierSurface::InitGeometry(const DxDevice& device)
{
	if (!m_patchVertexBuffer)
	{
		UINT vertexCount = m_gridPointsU * m_gridPointsV;
		m_patchVertexBuffer = device.CreateDynamicVertexBuffer<VertexPositionUV>(vertexCount);
	}

	if (!m_polylineVertexBuffer)
	{
		UINT vertexCount = m_gridPointsU * m_gridPointsV;
		m_polylineVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(vertexCount);
	}

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
	//m_patchVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.controlPoints);
	//m_polylineVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.patchVertices.value());
	m_patchVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.patchVertices.value_or(std::vector<VertexPositionUV>()));
	m_polylineVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.controlPoints);

	m_patchIndexCount = static_cast<UINT>(precalculatedData.patchIndices.size());
	m_patchIndexBuffer = device.CreateIndexBuffer(precalculatedData.patchIndices);

	std::vector<unsigned int> lineIndices = GenerateLineIndices();
	m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);

	m_isDirty = false;
}

void BezierSurface::UpdateVertices(const DxDevice& device)
{
	if (!m_isDirty)
		return;

	UINT vertexCount = m_gridPointsU * m_gridPointsV;

	std::vector<VertexPosition> polyPositions;
	std::vector<VertexPositionUV> patchPositions;
	polyPositions.reserve(vertexCount);
	patchPositions.reserve(vertexCount);

	for (unsigned int v = 0; v < m_gridPointsV; v++)
	{
		float uv_v = static_cast<float>(v) / (m_gridPointsV - 1);
		for (unsigned int u = 0; u < m_gridPointsU; ++u)
		{
			float uv_u = static_cast<float>(u) / (m_gridPointsU - 1);
			unsigned int idx = GetControlPointIndex(u, v);

			if (auto pt = m_controlPoints[idx].lock())
			{
				polyPositions.push_back({ pt->m_position.x, pt->m_position.y, pt->m_position.z });
				patchPositions.push_back({ pt->m_position.x, pt->m_position.y, pt->m_position.z, uv_u, uv_v});
			}
			else
			{
				polyPositions.push_back({ 0.0f, 0.0f, 0.0f });
				patchPositions.push_back({ 0.0f, 0.0f, 0.0f, uv_u, uv_v });
			}
		}
	}
	device.UpdateBuffer(m_polylineVertexBuffer, polyPositions.data(), static_cast<UINT>(polyPositions.size()) * sizeof(VertexPosition));
	device.UpdateBuffer(m_patchVertexBuffer, patchPositions.data(), static_cast<UINT>(patchPositions.size()) * sizeof(VertexPositionUV));

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