#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;

unsigned int BezierSurface::s_nextId = 0;

BezierSurface::BezierSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview)
	: Surface(uSeg, vSeg, shape, isPreview ? "Preview Surface" : "Surface C0 - " + std::to_string(s_nextId++))
	
{
	if (!isPreview)
		AssignGlobalID();
}

void BezierSurface::Commit()
{
	AssignGlobalID();
	name = "Surface C0 - " + std::to_string(s_nextId++);
}

void BezierSurface::InitGeometry(const DxDevice& device)
{
	unsigned int pointsU = GetGridPointsU();
	unsigned int pointsV = GetGridPointsV();
	UINT vertexCount = pointsU * pointsV;

	m_patchVertexBuffer = m_polylineVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(vertexCount);	

	std::vector<unsigned int> lineIndices = GenerateLineIndices();
	m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);

	std::vector<unsigned int> patchIndices = GeneratePatchIndices();
	m_patchIndexCount = static_cast<UINT>(patchIndices.size());
	m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
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

SurfaceGenerationResult BezierSurface::CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device)
{
	auto surface = std::make_unique<BezierSurface>(segU, segV, SurfaceShape::Flat, true);
	std::vector<std::shared_ptr<Point>> generatedPoints;

	int pointsU = 3 * segU + 1;
	int pointsV = 3 * segV + 1;
	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u) / (pointsU - 1);
			float3 pos{ uParam * width + center.x, center.y, vParam * length + center.z };
			auto pt = std::make_shared<Point>(pos, true, true);
			generatedPoints.push_back(pt);
			surface->m_controlPoints.push_back(pt);
		}
	}

	surface->InitGeometry(device);
	return { std::move(surface), std::move(generatedPoints) };
}

SurfaceGenerationResult BezierSurface::CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device)
{
	auto surface = std::make_unique<BezierSurface>(segU, segV, SurfaceShape::Cylinder, true);
	std::vector<std::shared_ptr<Point>> generatedPoints;

	int pointsU = 3 * segU;
	int pointsV = 3 * segV + 1;

	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u) / (pointsU);
			float dTheta = 2.0f * std::numbers::pi_v<float> / segU;
			float L = radius * (4.0f / 3.0f) * std::tan(dTheta / 4.0f);

			constexpr float startAngle = -std::numbers::pi_v<float> / 2.0f;
			int patchIndex = u / 3;
			int pointType = u % 3;
			float angle = patchIndex * dTheta + startAngle;

			float cx, cy;
			if (pointType == 0) // Anchor Point (On the circle)
			{
				cx = radius * std::cos(angle);
				cy = radius * std::sin(angle);
			}
			else if (pointType == 1) // Forward Tangent Handle (Pushed out)
			{
				cx = radius * std::cos(angle) - L * std::sin(angle);
				cy = radius * std::sin(angle) + L * std::cos(angle);
			}
			else // Backward Tangent Handle (Pushed out from the next anchor)
			{
				float nextAngle = (patchIndex + 1) * dTheta + startAngle;
				cx = radius * std::cos(nextAngle) + L * std::sin(nextAngle);
				cy = radius * std::sin(nextAngle) - L * std::cos(nextAngle);
			}

			float3 pos = { cx + center.x, (cy + radius) + center.y, vParam * height + center.z };

			auto pt = std::make_shared<Point>(pos, true, true);
			generatedPoints.push_back(pt);
			surface->m_controlPoints.push_back(pt);
		}
	}

	surface->InitGeometry(device);
	return { std::move(surface), std::move(generatedPoints) };
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