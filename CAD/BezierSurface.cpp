#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"

unsigned int BezierSurface::s_nextId = 0;

BezierSurface::BezierSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview)
	: SceneObject(isPreview ? "Preview Surface" : "Surface C0 - " + std::to_string(s_nextId++), ObjectType::BezierSurface),
	segmentsU(uSeg), segmentsV(vSeg), shapeType(shape)
{}

void BezierSurface::Commit()
{
	name = "Surface C0 - " + std::to_string(s_nextId++);
}

std::shared_ptr<Point> BezierSurface::GetPoint(int u, int v) const
{
	return m_controlPoints[GetPointIndex(u, v)].lock();
}

//void BezierSurface::UpdatePatches(const DxDevice& device)
//{
//	if (!m_isDirty)
//		return;
//
//	std::vector<VertexPosition> patchVertices;
//
//	for (int patchV = 0; patchV < segmentsV; patchV++)
//	{
//		for (int patchU = 0; patchU < segmentsU; patchU++)
//		{
//			for (int v = 0; v < 4; v++)
//			{
//				for (int u = 0; u < 4; u++)
//				{
//					if (auto point = GetPoint(patchU * 3 + u, patchV * 3 + v))
//						patchVertices.push_back({ point->m_position.x, point->m_position.y, point->m_position.z });
//					else
//						patchVertices.push_back({ 0.0f, 0.0f, 0.0f });
//				}
//			}
//		}
//	}
//
//	UpdateDynamicBuffer(device, m_patchBuffer, m_patchBufferCapacity, patchVertices);
//	m_patchVertexCount = static_cast<UINT>(patchVertices.size());
//	//m_isDirty = false;
//}

void BezierSurface::InitGeometry(const DxDevice& device)
{
	unsigned int pointsU = GetPhysicalPointsU();
	unsigned int pointsV = GetPhysicalPointsV();
	UINT vertexCount = pointsU * pointsV;

	m_vertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(vertexCount);

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

	unsigned int pointsU = GetPhysicalPointsU();
	unsigned int pointsV = GetPhysicalPointsV();
	UINT vertexCount = pointsU * pointsV;

	std::vector<VertexPosition> positions;
	positions.reserve(vertexCount);

	for (unsigned int v = 0; v < pointsV; v++)
	{
		for (unsigned int u = 0; u < pointsU; ++u)
		{
			unsigned int idx = GetPointIndex(u, v);

			if (auto pt = m_controlPoints[idx].lock())
				positions.push_back({ pt->m_position.x, pt->m_position.y, pt->m_position.z });
			else
				positions.push_back({ 0.0f, 0.0f, 0.0f });
		}
	}

	device.UpdateBuffer(m_vertexBuffer, positions.data(), static_cast<UINT>(positions.size()) * sizeof(VertexPosition));

	//m_isDirty = false;
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

std::vector<unsigned int> BezierSurface::GenerateLineIndices() const
{
	std::vector<unsigned int> indices;

	int pointsU = GetPhysicalPointsU();
	int pointsV = GetPhysicalPointsV();
	int logicalPointsU = (shapeType == SurfaceShape::Cylinder) ? pointsU + 1 : pointsU;

	// 1. Horizontal lines (U direction)
	for (int v = 0; v < pointsV; v++)
	{
		for (int u = 0; u < logicalPointsU - 1; u++)
		{
			indices.push_back(GetPointIndex(u, v));
			indices.push_back(GetPointIndex(u + 1, v));
		}
	}

	// 2. Vertical lines (V direction)
	// We only loop to 'pointsU' here (physical points) so we don't draw the seam twice.
	for (int u = 0; u < pointsU; u++)
	{
		for (int v = 0; v < pointsV - 1; ++v)
		{
			indices.push_back(GetPointIndex(u, v));
			indices.push_back(GetPointIndex(u, v + 1));
		}
	}

	return indices;
}

std::vector<unsigned int> BezierSurface::GeneratePatchIndices() const
{
	std::vector<unsigned int> indices;
	indices.reserve(static_cast<size_t>(segmentsU) * segmentsV * 16);

	// Iterate over every patch
	for (int patchV = 0; patchV < segmentsV; patchV++)
		for (int patchU = 0; patchU < segmentsU; patchU++)
		{
			// Each patch grabs a 4x4 block of control points
			for (int v = 0; v < 4; v++)
				for (int u = 0; u < 4; u++)
					indices.push_back(GetPointIndex(patchU * 3 + u, patchV * 3 + v));
		}

	return indices;
}
unsigned int BezierSurface::GetPhysicalPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);
}
unsigned int BezierSurface::GetPhysicalPointsV() const
{
	return 3 * segmentsV + 1;
}
unsigned int BezierSurface::GetPointIndex(int u, int v) const
{
	unsigned int pointsU = GetPhysicalPointsU();
	int wrappedU = (shapeType == SurfaceShape::Cylinder) ? (u % pointsU) : u;
	return static_cast<unsigned int>(v * pointsU + wrappedU);
}
//void BezierSurface::UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data)
//{
//	if (data.size() > capacity)
//	{
//		capacity = std::max({ static_cast<UINT>(data.size()), static_cast<UINT>(capacity * 1.5), 16u });
//		buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
//	}
//	device.UpdateBuffer(buffer, data.data(), static_cast<UINT>(data.size()) * sizeof(VertexPosition));
//}