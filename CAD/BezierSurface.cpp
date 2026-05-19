#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;

unsigned int BezierSurface::s_nextId = 0;

BezierSurface::BezierSurface(int uSeg, int vSeg, SurfaceShape shape)
	: Surface(uSeg, vSeg, shape, "Surface C0 - " + std::to_string(s_nextId++))
{}

void BezierSurface::InitGeometry(const DxDevice& device)
{

	if (!m_patchVertexBuffer)
	{
		UINT vertexCount = GetGridPointsU() * GetGridPointsV();
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

	unsigned int pointsU = GetGridPointsU();
	unsigned int pointsV = GetGridPointsV();
	UINT vertexCount = pointsU * pointsV;

	std::vector<VertexPosition> positions;
	positions.reserve(vertexCount);

	for (unsigned int v = 0; v < pointsV; v++)
	{
		for (unsigned int u = 0; u < pointsU; ++u)
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

unsigned int BezierSurface::GetGridPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);
}

unsigned int BezierSurface::GetGridPointsV() const
{
	return 3 * segmentsV + 1;
}

unsigned int BezierSurface::GetPatchDataIndex(int u, int v) const
{
	return GetControlPointIndex(u, v);
}