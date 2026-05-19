#include "pch.h"
#include "BSplineSurface.h"
#include "DxDevice.h"

using namespace MathLib;


unsigned int BSplineSurface::s_nextId = 0;

BSplineSurface::BSplineSurface(int uSeg, int vSeg, SurfaceShape shape)
	: Surface(uSeg, vSeg, shape, "Surface C2 - " + std::to_string(s_nextId++))
{}

unsigned int BSplineSurface::GetBernsteinPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);
}
unsigned int BSplineSurface::GetBernsteinPointsV() const
{
	return 3 * segmentsV + 1;
}

unsigned int BSplineSurface::GetBernsteinIndex(int u, int v) const
{
	unsigned int pointsU = GetBernsteinPointsU();
	int wrappedU = (shapeType == SurfaceShape::Cylinder) ? (u % pointsU) : u;
	return static_cast<unsigned int>(v * pointsU + wrappedU);
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

void BSplineSurface::InitGeometry(const DxDevice& device)
{
	// 1. De Boor Buffer (for wireframe)
	if (!m_polylineVertexBuffer)
	{
		UINT deBoorCount = GetGridPointsU() * GetGridPointsV();
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
	UINT deBoorCount = GetGridPointsU() * GetGridPointsV();
	std::vector<VertexPosition> deBoorPositions(deBoorCount);

	for (unsigned int v = 0; v < GetGridPointsV(); v++)
	{
		for (unsigned int u = 0; u < GetGridPointsU(); ++u)
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
	for (int patchV = 0; patchV < segmentsV; patchV++)
	{
		for (int patchU = 0; patchU < segmentsU; patchU++)
		{
			ConvertPatchToBernstein(patchU, patchV, bernsteinPositions);
		}
	}
	device.UpdateBuffer(m_patchVertexBuffer, bernsteinPositions.data(), bernsteinCount * sizeof(VertexPosition));

	m_isDirty = false;
}

unsigned int BSplineSurface::GetGridPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? segmentsU : (segmentsU + 3);
}

unsigned int BSplineSurface::GetGridPointsV() const
{
	return segmentsV + 3;
}

unsigned int BSplineSurface::GetPatchDataIndex(int u, int v) const
{
	return GetBernsteinIndex(u, v);
}
