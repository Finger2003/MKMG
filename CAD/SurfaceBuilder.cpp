#include "pch.h"
#include "SurfaceBuilder.h"
#include "SplineMath.h"
#include "DxDevice.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
using namespace MathLib;

void SurfaceBuilder::UpdateGeometry(const DxDevice& device, SurfaceType type, SurfaceShape shape, int segU, int segV, float d1, float d2, const float3& centerPos)
{
	bool topologyChanged = (shape != surfaceShape) || (type != surfaceType)
		|| (segU != segmentsU) || (segV != segmentsV);

	surfaceType = type;
	surfaceShape = shape;
	segmentsU = segU;
	segmentsV = segV;
	dim1 = d1;
	dim2 = d2;
	center = centerPos;

	rawPoints.clear();
	const std::vector<VertexPosition>* dataToUpload = nullptr;
	if (surfaceType == SurfaceType::C0)
	{
		if (surfaceShape == SurfaceShape::Flat)
			GenerateC0Flat();
		else
			GenerateC0Cylinder();
		dataToUpload = &rawPoints;
	}
	else
	{
		if (surfaceShape == SurfaceShape::Flat)
			GenerateC2Flat();
		else
			GenerateC2Cylinder();

		bernsteinPoints = GenerateC2BernsteinPoints();
		dataToUpload = &bernsteinPoints;
	}

	UpdateVertexBuffer(device, m_patchVertexBuffer, m_patchVertexBufferCapacity, *dataToUpload);

	if (topologyChanged)
	{
		patchIndices = GeneratePatchIndices();
		m_patchIndexCount = static_cast<UINT>(patchIndices.size());
		m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
	}
}

SurfaceGenerationResult SurfaceBuilder::Build(const DxDevice& device) const
{
	std::unique_ptr<Surface> surface;
	if (surfaceType == SurfaceType::C0)
		surface = std::make_unique<BezierSurface>(segmentsU, segmentsV, surfaceShape);
	else
		surface = std::make_unique<BSplineSurface>(segmentsU, segmentsV, surfaceShape);
	

	surface->m_linesPerSegmentU = linesPerSegmentU;
	surface->m_linesPerSegmentV = linesPerSegmentV;

	std::vector<std::shared_ptr<Point>> generatedPoints;
	generatedPoints.reserve(rawPoints.size());

	for (const VertexPosition& pos : rawPoints)
	{
		float3 floatPos{ pos.x, pos.y, pos.z };
		auto pt = std::make_shared<Point>(floatPos, true);
		generatedPoints.push_back(pt);
		surface->m_controlPoints.push_back(pt);
	}

	if (surfaceType == SurfaceType::C0)
	{
		surface->m_patchVertexBuffer = device.CreateDynamicVertexBuffer(rawPoints);
		surface->m_polylineVertexBuffer = surface->m_patchVertexBuffer;
	}
	else
	{
		surface->m_patchVertexBuffer = device.CreateDynamicVertexBuffer(bernsteinPoints);
		surface->m_polylineVertexBuffer = device.CreateDynamicVertexBuffer(rawPoints);
	}

	surface->m_patchIndexCount = static_cast<UINT>(patchIndices.size());
	surface->m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);

	std::vector<unsigned int> lineIndices = surface->GenerateLineIndices();
	surface->m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	surface->m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);

	return { std::move(surface), std::move(generatedPoints) };
}

void SurfaceBuilder::UpdateVertexBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data)
{
	if (data.size() > capacity)
	{
		capacity = std::max(static_cast<UINT>(data.size()), static_cast<UINT>(capacity * 1.5));
		buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
	}
	device.UpdateBuffer(buffer, data.data(), static_cast<size_t>(data.size()) * sizeof(VertexPosition));
}

void SurfaceBuilder::GenerateC0Flat()
{
	int pointsU = 3 * segmentsU + 1;
	int pointsV = 3 * segmentsV + 1;
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);
	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u) / (pointsU - 1);
			rawPoints.push_back({
				uParam * width + center.x,
				center.y,
				vParam * length + center.z
				});
		}
	}
}

void SurfaceBuilder::GenerateC0Cylinder()
{
	int pointsU = 3 * segmentsU;
	int pointsV = 3 * segmentsV + 1;
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	float dTheta = 2.0f * std::numbers::pi_v<float> / segmentsU;
	float L = radius * (4.0f / 3.0f) * std::tan(dTheta / 4.0f);
	constexpr float startAngle = -std::numbers::pi_v<float> / 2.0f;

	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u) / (pointsU);
			int patchIndex = u / 3;
			int pointType = u % 3;
			float angle = patchIndex * dTheta + startAngle;

			float cx, cy;
			if (pointType == 0)
			{
				cx = radius * std::cos(angle);
				cy = radius * std::sin(angle);
			}
			else if (pointType == 1)
			{
				cx = radius * std::cos(angle) - L * std::sin(angle);
				cy = radius * std::sin(angle) + L * std::cos(angle);
			}
			else
			{
				float nextAngle = (patchIndex + 1) * dTheta + startAngle;
				cx = radius * std::cos(nextAngle) + L * std::sin(nextAngle);
				cy = radius * std::sin(nextAngle) - L * std::cos(nextAngle);
			}

			rawPoints.push_back({
				cx + center.x,
				(cy + radius) + center.y,
				vParam * length + center.z
				});
		}
	}
}

void SurfaceBuilder::GenerateC2Flat()
{
	int pointsU = segmentsU + 3;
	int pointsV = segmentsV + 3;
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v - 1) / segmentsV;
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u - 1) / segmentsU;
			rawPoints.push_back({
				uParam * width + center.x,
				center.y,
				vParam * length + center.z,
				});
		}
	}
}

void SurfaceBuilder::GenerateC2Cylinder()
{
	int pointsU = segmentsU;
	int pointsV = segmentsV + 3;
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	float dTheta = 2.0f * std::numbers::pi_v<float> / segmentsU;
	float R_deBoor = radius * (3.0f / (2.0f + std::cos(dTheta)));
	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v - 1) / segmentsV;
		for (int u = 0; u < pointsU; u++)
		{
			float angle = u * dTheta;
			rawPoints.push_back({
				R_deBoor * std::cos(angle) + center.x,
				R_deBoor * std::sin(angle) + center.y + radius,
				vParam * length + center.z
				});
		}
	}
}

std::vector<VertexPosition> SurfaceBuilder::GenerateC2BernsteinPoints() const
{
	int bernU = (surfaceShape == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);
	int bernV = 3 * segmentsV + 1;
	int deBoorU = (surfaceShape == SurfaceShape::Cylinder) ? segmentsU : (segmentsU + 3);
	std::vector<VertexPosition> bernsteinGrid(bernU * bernV);

	auto getWrappedU = [&](int u, int maxU) -> int {
			return (surfaceShape == SurfaceShape::Cylinder) ? (u % maxU) : u;
		};
	auto getDeBoorPoint = [&](int u, int v) -> VertexPosition {
		int wrappedU = getWrappedU(u, deBoorU);
		return rawPoints[static_cast<size_t>(v) * deBoorU + wrappedU];
		};
	auto getBernsteinIndex = [&](int u, int v) -> int {
		int wrappedU = getWrappedU(u, bernU);
		return v * bernU + wrappedU;
		};

	for (int patchV = 0; patchV < segmentsV; patchV++)
	{
		for (int patchU = 0; patchU < segmentsU; patchU++)
		{
			Vec3f P[4][4];
			for (int v = 0; v < 4; v++)
				for (int u = 0; u < 4; u++)
					P[v][u] = ToVec3f(getDeBoorPoint(patchU + u, patchV + v));
			Vec3f Q[4][4];
			for (int v = 0; v < 4; v++)
				for (int u = 0; u < 4; u++)
					Q[v][u] = SplineMath::EvaluateBSpline1D(P[v][0], P[v][1], P[v][2], P[v][3], u);
			for (int v = 0; v < 4; v++)
				for (int u = 0; u < 4; u++)
				{
					Vec3f B = SplineMath::EvaluateBSpline1D(Q[0][u], Q[1][u], Q[2][u], Q[3][u], v);
					int bIdx = getBernsteinIndex(patchU * 3 + u, patchV * 3 + v);
					bernsteinGrid[bIdx] = ToVertexPosition(B);
				}
		}
	}

	return bernsteinGrid;
}

std::vector<unsigned int> SurfaceBuilder::GeneratePatchIndices() const
{
	std::vector<unsigned int> indices;
	indices.reserve(static_cast<size_t>(segmentsU) * segmentsV * 16);
	int bernU = (surfaceShape == SurfaceShape::Cylinder) ? (3 * segmentsU) : (3 * segmentsU + 1);

	auto getIndex = [&](int u, int v) {
		int wrappedU = (surfaceShape == SurfaceShape::Cylinder) ? (u % bernU) : u;
		return v * bernU + wrappedU;
		};

	for (int patchV = 0; patchV < segmentsV; patchV++)
	{
		for (int patchU = 0; patchU < segmentsU; patchU++)
		{
			for (int v = 0; v < 4; v++)
			{
				for (int u = 0; u < 4; u++)
				{
					indices.push_back(getIndex(patchU * 3 + u, patchV * 3 + v));
				}
			}
		}
	}
	return indices;
}
