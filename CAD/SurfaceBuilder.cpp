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
	//const std::vector<VertexPosition>* dataToUpload = nullptr;
	if (surfaceType == SurfaceType::C0)
	{
		if (surfaceShape == SurfaceShape::Flat)
			GenerateC0Flat();
		else
			GenerateC0Cylinder();

		patchPointsUV = GenerateC0PatchPointsUV();
		//dataToUpload = &rawPoints;
	}
	else
	{
		if (surfaceShape == SurfaceShape::Flat)
			GenerateC2Flat();
		else
			GenerateC2Cylinder();

		patchPointsUV = GenerateC2BernsteinPointsUV();

		//bernsteinPoints = GenerateC2BernsteinPoints();
		//dataToUpload = &bernsteinPoints;
	}

	UpdateVertexBuffer(device, m_patchVertexBuffer, m_patchVertexBufferCapacity, patchPointsUV);

	if (topologyChanged)
	{
		patchIndices = GeneratePatchIndices();
		m_patchIndexCount = static_cast<UINT>(patchIndices.size());
		m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
	}
}

SurfaceGenerationResult SurfaceBuilder::Build(const DxDevice& device) const
{
	unsigned int gridV = static_cast<unsigned int>(indexGrid.size());
	unsigned int gridU = indexGrid.empty() ? 0 : static_cast<unsigned int>(indexGrid[0].size());

	std::vector<std::shared_ptr<Point>> generatedPoints(m_uniquePointsCount);
	for (unsigned int v = 0; v < gridV; ++v)
	{
		for (unsigned int u = 0; u < gridU; ++u)
		{
			int id = indexGrid[v][u];
			if (!generatedPoints[id])
			{
				size_t index = static_cast<size_t>(v * gridU + u);
				const auto& pos = rawPoints[index];
				generatedPoints[id] = std::make_shared<Point>(float3{ pos.x, pos.y, pos.z });
			}
		}
	}

	std::vector<std::weak_ptr<Point>> surfaceControlPoints;
	surfaceControlPoints.reserve(static_cast<size_t>(gridU) * gridV);
	for (unsigned int v = 0; v < gridV; ++v)
	{
		for (unsigned int u = 0; u < gridU; ++u)
		{
			surfaceControlPoints.push_back(generatedPoints[indexGrid[v][u]]);
		}
	}

	std::unique_ptr<Surface> surface;
	if (surfaceType == SurfaceType::C0)
		surface = std::make_unique<BezierSurface>(gridU, gridV, linesPerSegmentU, linesPerSegmentV, std::move(surfaceControlPoints));
	else
		surface = std::make_unique<BSplineSurface>(gridU, gridV, linesPerSegmentU, linesPerSegmentV, std::move(surfaceControlPoints));

	for(auto& pt : surface->m_controlPoints)
		if (auto sp = pt.lock())
			sp->m_surfaceLockCount++;

	PrecalculatedSurfaceData precalcData
	{
		.controlPoints = rawPoints,
		.patchVertices = patchPointsUV,
		.patchIndices = patchIndices
	};

	surface->InitGeometry(device, precalcData);
	return { std::move(surface), std::move(generatedPoints) };
}

std::vector<VertexPositionUV> SurfaceBuilder::GenerateC0PatchPointsUV() const
{
	std::vector<VertexPositionUV> pts;
	pts.reserve(rawPoints.size());
	int pointsU = 3 * segmentsU + 1;
	int pointsV = 3 * segmentsV + 1;

	for (int v = 0; v < pointsV; v++)
	{
		float uv_v = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uv_u = static_cast<float>(u) / (pointsU - 1);
			size_t idx = static_cast<size_t>(v * pointsU + u);
			VertexPosition vp = rawPoints[idx];
			pts.push_back({ vp.x, vp.y, vp.z, uv_u, uv_v });
		}
	}
	return pts;
}

std::vector<VertexPositionUV> SurfaceBuilder::GenerateC2BernsteinPointsUV() const
{
	int bernU = 3 * segmentsU + 1;
	int bernV = 3 * segmentsV + 1;
	int deBoorU = segmentsU + 3;

	std::vector<VertexPositionUV> bernsteinGrid(bernU * bernV);

	auto getDeBoorPoint = [&](int u, int v) -> VertexPosition {
		return rawPoints[static_cast<size_t>(v) * deBoorU + u];
		};
	auto getBernsteinIndex = [&](int u, int v) -> int {
		return v * bernU + u;
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
					int bu = patchU * 3 + u;
					int bv = patchV * 3 + v;
					int bIdx = getBernsteinIndex(patchU * 3 + u, patchV * 3 + v);
					float uv_u = static_cast<float>(bu) / (bernU - 1);
					float uv_v = static_cast<float>(bv) / (bernV - 1);
					bernsteinGrid[bIdx] = { B.x, B.y, B.z, uv_u, uv_v };
				}
		}
	}

	return bernsteinGrid;
}


void SurfaceBuilder::GenerateC0Flat()
{
	int pointsU = 3 * segmentsU + 1;
	int pointsV = 3 * segmentsV + 1;
	indexGrid.assign(pointsV, std::vector<int>(pointsU, 0));
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	int uniqueIdCounter = 0;
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
			indexGrid[v][u] = uniqueIdCounter++;
		}
	}
	m_uniquePointsCount = uniqueIdCounter;
}

void SurfaceBuilder::GenerateC0Cylinder()
{
	int pointsU = 3 * segmentsU + 1;
	int pointsV = 3 * segmentsV + 1;
	int uniqueU = 3 * segmentsU;
	indexGrid = std::vector<std::vector<int>>(pointsV, std::vector<int>(pointsU));
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	float dTheta = 2.0f * std::numbers::pi_v<float> / segmentsU;
	float L = radius * (4.0f / 3.0f) * std::tan(dTheta / 4.0f);
	constexpr float startAngle = -std::numbers::pi_v<float> / 2.0f;

	int uniqueIdCounter = 0;
	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		int firstIdInRow = uniqueIdCounter;
		for (int u = 0; u < pointsU; u++)
		{
			int wrappedU = u % uniqueU;
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

			rawPoints.push_back({ cx + center.x, cy + radius + center.y, vParam * length + center.z });

			if (u < uniqueU) 
				indexGrid[v][u] = uniqueIdCounter++;
			else
				indexGrid[v][u] = indexGrid[v][wrappedU];
		}
	}

	m_uniquePointsCount = uniqueIdCounter;
}

void SurfaceBuilder::GenerateC2Flat()
{
	int pointsU = segmentsU + 3;
	int pointsV = segmentsV + 3;
	indexGrid.assign(pointsV, std::vector<int>(pointsU, 0));
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	int uniqueIdCounter = 0;
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
			indexGrid[v][u] = uniqueIdCounter++;
		}
	}

	m_uniquePointsCount = uniqueIdCounter;
}

void SurfaceBuilder::GenerateC2Cylinder()
{
	int pointsU = segmentsU + 3;
	int pointsV = segmentsV + 3;
	int uniqueU = segmentsU;

	indexGrid.assign(pointsV, std::vector<int>(pointsU, 0));
	rawPoints.reserve(static_cast<size_t>(pointsU) * pointsV);

	constexpr float startAngle = -std::numbers::pi_v<float> / 2.0f;
	float dTheta = 2.0f * std::numbers::pi_v<float> / segmentsU;
	float R_deBoor = radius * (3.0f / (2.0f + std::cos(dTheta)));

	int uniqueIdCounter = 0;
	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v - 1) / segmentsV;
		int firstIdInRow = uniqueIdCounter;

		for (int u = 0; u < pointsU; u++)
		{
			int wrappedU = u % uniqueU;
			float angle = u * dTheta + startAngle;
			float cx = R_deBoor * std::cos(angle);
			float cy = R_deBoor * std::sin(angle);
			rawPoints.push_back({
				cx + center.x,
				cy + center.y + radius,
				vParam * length + center.z
				});

			if (u < uniqueU) 
				indexGrid[v][u] = uniqueIdCounter++;
			else 
				indexGrid[v][u] = indexGrid[v][wrappedU];
		}
	}
	m_uniquePointsCount = uniqueIdCounter;
}

std::vector<VertexPosition> SurfaceBuilder::GenerateC2BernsteinPoints() const
{
	int bernU = 3 * segmentsU + 1;
	int bernV = 3 * segmentsV + 1;
	int deBoorU = segmentsU + 3;

	std::vector<VertexPosition> bernsteinGrid(bernU * bernV);

	auto getDeBoorPoint = [&](int u, int v) -> VertexPosition {
		return rawPoints[static_cast<size_t>(v) * deBoorU + u];
		};
	auto getBernsteinIndex = [&](int u, int v) -> int {
		return v * bernU + u;
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
	int widthU = 3 * segmentsU + 1;

	auto getIndex = [&](int u, int v) -> unsigned int {
		return static_cast<unsigned int>(v * widthU + u);
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

std::vector<unsigned int> SurfaceBuilder::GenerateLineIndices() const
{
	std::vector<unsigned int> indices;
	if (indexGrid.empty()) 
		return indices;
	int pointsV = static_cast<int>(indexGrid.size());
	int pointsU = static_cast<int>(indexGrid[0].size());

	auto getIndex = [&](int u, int v) -> unsigned int {
		return static_cast<unsigned int>(v * pointsU + u);
		};

	for (int v = 0; v < pointsV; v++)
	{
		for (int u = 0; u < pointsU - 1; u++)
		{
			indices.push_back(getIndex(u, v));
			indices.push_back(getIndex(u + 1, v));
		}
	}

	for (int u = 0; u < pointsU; u++)
	{
		for (int v = 0; v < pointsV - 1; ++v)
		{
			indices.push_back(getIndex(u, v));
			indices.push_back(getIndex(u, v + 1));
		}
	}

	return indices;
}
