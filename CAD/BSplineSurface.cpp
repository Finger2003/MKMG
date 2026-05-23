#include "pch.h"
#include "BSplineSurface.h"
#include "DxDevice.h"

using namespace MathLib;

BSplineSurface::BSplineSurface(int uGrid, int vGrid, int linesPerSegmentU, int linesPerSegmentV, std::vector<std::weak_ptr<Point>> controlPoints)
	: Surface(uGrid, vGrid, linesPerSegmentU, linesPerSegmentV, std::move(controlPoints), "Surface C2 - " + std::to_string(s_nextId++))
{}

BSplineSurface::BSplineSurface(unsigned int id, uint2 grid, uint2 samples, std::vector<std::weak_ptr<Point>> && controlPoints, std::optional<ParsedNameData> && nameData)
	: Surface(id, nameData ? std::move(nameData->name) : "Surface C2 - " + std::to_string(s_nextId++), grid.u, grid.v, samples.u, samples.v, std::move(controlPoints))
{
	if (nameData)
		AdvanceCounter(nameData->index);
}

unsigned int BSplineSurface::GetBernsteinPointsU() const
{
	return 3 * (m_gridPointsU - 3) + 1;
}
unsigned int BSplineSurface::GetBernsteinPointsV() const
{
	return 3 * (m_gridPointsV - 3) + 1;
}

unsigned int BSplineSurface::GetBernsteinIndex(int u, int v) const
{
	return static_cast<unsigned int>(v * GetBernsteinPointsU() + u);
}

unsigned int BSplineSurface::GetSegmentsU() const
{
	return m_gridPointsU - 3;
}

unsigned int BSplineSurface::GetSegmentsV() const
{
	return m_gridPointsV - 3;
}

void BSplineSurface::ConvertPatchToBernstein(int patchU, int patchV, std::vector<VertexPosition>& bernsteinGrid) const
{
	Vec3f P[4][4];
	for (int v = 0; v < 4; v++)
		for (int u = 0; u < 4; u++)
		{
			if (auto pt = GetPoint(patchU + u, patchV + v))
				P[v][u] = pt->m_position.ToVec3f();
			else
				P[v][u] = { 0,0,0 };
		}

	Vec3f Q[4][4];
	for (int v = 0; v < 4; v++)
		for (int u = 0; u < 4; u++)
			Q[v][u] = Evaluate1D(P[v][0], P[v][1], P[v][2], P[v][3], u);

	for (int v = 0; v < 4; v++)
		for (int u = 0; u < 4; u++)
		{
			Vec3f B = Evaluate1D(Q[0][u], Q[1][u], Q[2][u], Q[3][u], v);
			unsigned int bIdx = GetBernsteinIndex(patchU * 3 + u, patchV * 3 + v);
			bernsteinGrid[bIdx] = { B.x, B.y, B.z };
		}
}

MathLib::Vec3f BSplineSurface::Evaluate1D(MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3, int index) const
{
	switch (index)
	{
	case 0:
		return (p0 + 4.0f * p1 + p2) / 6.0f;
	case 1:
		return (4.0f * p1 + 2.0f * p2) / 6.0f;
	case 2:
		return (2.0f * p1 + 4.0f * p2) / 6.0f;
	case 3:
		return (p1 + 4.0f * p2 + p3) / 6.0f;
	}
}



void BSplineSurface::InitGeometry(const DxDevice& device, const PrecalculatedSurfaceData& precalculatedData)
{
	m_patchVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.patchVertices.value_or(std::vector<VertexPosition>()));
	m_patchIndexCount = static_cast<UINT>(precalculatedData.patchIndices.size());
	m_patchIndexBuffer = device.CreateIndexBuffer(precalculatedData.patchIndices);

	m_polylineVertexBuffer = device.CreateDynamicVertexBuffer(precalculatedData.controlPoints);
	std::vector<unsigned int> lineIndices = GenerateLineIndices();
	m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);

	m_isDirty = false;
}

void BSplineSurface::InitGeometry(const DxDevice& device)
{
	// 1. De Boor Buffer (for wireframe)
	if (!m_polylineVertexBuffer)
	{
		UINT deBoorCount = m_gridPointsU * m_gridPointsV;
		m_polylineVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(deBoorCount);
	}
	if (!m_polylineIndexBuffer)
	{
		std::vector<unsigned int> lineIndices = GenerateLineIndices();
		m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
		m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);
	}
	// 2. Bernstein Buffer (for patches)
	if (!m_patchVertexBuffer)
	{
		UINT bernsteinCount = GetBernsteinPointsU() * GetBernsteinPointsV();
		m_patchVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(bernsteinCount);
	}

	if (!m_patchIndexBuffer)
	{
		std::vector<unsigned int> patchIndices = GeneratePatchIndices();
		m_patchIndexCount = static_cast<UINT>(patchIndices.size());
		m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
	}
}

void BSplineSurface::UpdateVertices(const DxDevice& device)
{
	if (!m_isDirty)
		return;

	// 1. Upload De Boor Points (Wireframe)
	UINT deBoorCount = m_gridPointsU * m_gridPointsV;
	std::vector<VertexPosition> deBoorPositions(deBoorCount);

	for (unsigned int v = 0; v < m_gridPointsV; v++)
	{
		for (unsigned int u = 0; u < m_gridPointsU; ++u)
		{
			unsigned int idx = GetControlPointIndex(u, v);
			if (auto pt = m_controlPoints[idx].lock())
				deBoorPositions[idx] = { pt->m_position.x, pt->m_position.y, pt->m_position.z };
		}
	}
	device.UpdateBuffer(m_polylineVertexBuffer, deBoorPositions.data(), deBoorCount * sizeof(VertexPosition));


	// 2. Convert and Upload Bernstein Points (Surface Patches)
	UINT bernsteinCount = GetBernsteinPointsU() * GetBernsteinPointsV();
	std::vector<VertexPosition> bernsteinPositions(bernsteinCount);
	for (int patchV = 0; patchV < GetSegmentsV(); patchV++)
	{
		for (int patchU = 0; patchU < GetSegmentsU(); patchU++)
		{
			ConvertPatchToBernstein(patchU, patchV, bernsteinPositions);
		}
	}
	device.UpdateBuffer(m_patchVertexBuffer, bernsteinPositions.data(), bernsteinCount * sizeof(VertexPosition));

	m_isDirty = false;
}

std::vector<unsigned int> BSplineSurface::GeneratePatchIndices() const
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
					indices.push_back(GetBernsteinIndex(patchU * 3 + u, patchV * 3 + v));
				}
			}
		}
	}
	return indices;
}

unsigned int BSplineSurface::GetPatchDataIndex(int u, int v) const
{
	return GetBernsteinIndex(u, v);
}
