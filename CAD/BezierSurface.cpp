#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"
using namespace MathLib;

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


unsigned int BSplineSurface::s_nextId = 0;

BSplineSurface::BSplineSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview)
	: SceneObject(isPreview ? "Preview Surface" : "Surface C2 - " + std::to_string(s_nextId++), ObjectType::BSplineSurface),
	segmentsU(uSeg), segmentsV(vSeg), shapeType(shape)
{}

void BSplineSurface::Commit()
{
	name = "Surface C2 - " + std::to_string(s_nextId++);
}

std::shared_ptr<Point> BSplineSurface::GetPoint(int u, int v) const
{
	return m_controlPoints[GetDeBoorIndex(u, v)].lock();
}

unsigned int BSplineSurface::GetDeBoorPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? segmentsU : (segmentsU + 3);
}

unsigned int BSplineSurface::GetDeBoorPointsV() const 
{ 
	return segmentsV + 3; 
}

unsigned int BSplineSurface::GetBernsteinPointsU() const
{
	return (shapeType == SurfaceShape::Cylinder) ? (3 * GetPatchesU()) : (3 * GetPatchesU() + 1);
}
unsigned int BSplineSurface::GetBernsteinPointsV() const 
{ 
	return 3 * GetPatchesV() + 1;
}

unsigned int BSplineSurface::GetPatchesU() const
{
	//return GetDeBoorPointsU();
	return (shapeType == SurfaceShape::Cylinder) ? segmentsU : (segmentsU + 2);
}

unsigned int BSplineSurface::GetPatchesV() const
{
	//return GetDeBoorPointsV();
	return segmentsV + 2;
}

unsigned int BSplineSurface::GetDeBoorIndex(int u, int v) const
{
	unsigned int pointsU = GetDeBoorPointsU();
	int wrappedU = (shapeType == SurfaceShape::Cylinder) ? (u % pointsU) : u;
	return static_cast<unsigned int>(v * pointsU + wrappedU);
}

unsigned int BSplineSurface::GetBernsteinIndex(int u, int v) const
{
	unsigned int pointsU = GetBernsteinPointsU();
	int wrappedU = (shapeType == SurfaceShape::Cylinder) ? (u % pointsU) : u;
	return static_cast<unsigned int>(v * pointsU + wrappedU);
}

void BSplineSurface::ConvertPatchToBernstein(int patchU, int patchV, const std::vector<std::vector<Vec3f>>& augGrid,std::vector<VertexPosition>& bernsteinGrid) const
{
	Vec3f P[4][4];
	for (int v = 0; v < 4; v++)
		for (int u = 0; u < 4; u++)
		{
			int readU = patchU + u;
			if (shapeType == SurfaceShape::Cylinder)
				readU = readU % GetDeBoorPointsU();
			int readV = patchV + v;
			P[v][u] = augGrid[readU][readV];
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
	UINT deBoorCount = GetDeBoorPointsU() * GetDeBoorPointsV();
	m_deBoorVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(deBoorCount);

	std::vector<unsigned int> lineIndices = GenerateLineIndices();
	m_polylineIndexCount = static_cast<UINT>(lineIndices.size());
	m_polylineIndexBuffer = device.CreateIndexBuffer(lineIndices);

	// 2. Bernstein Buffer (for patches)
	UINT bernsteinCount = GetBernsteinPointsU() * GetBernsteinPointsV();
	m_bernsteinVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(bernsteinCount);

	std::vector<unsigned int> patchIndices = GeneratePatchIndices();
	m_patchIndexCount = static_cast<UINT>(patchIndices.size());
	m_patchIndexBuffer = device.CreateIndexBuffer(patchIndices);
}

void BSplineSurface::UpdateVertices(const DxDevice& device)
{
	if (!m_isDirty) 
		return;

	// 1. Upload De Boor Points (Wireframe)
	UINT deBoorCount = GetDeBoorPointsU() * GetDeBoorPointsV();
	std::vector<VertexPosition> deBoorPositions(deBoorCount, { 0,0,0 });

	for (unsigned int v = 0; v < GetDeBoorPointsV(); v++)
	{
		for (unsigned int u = 0; u < GetDeBoorPointsU(); ++u)
		{
			unsigned int idx = GetDeBoorIndex(u, v);
			if (auto pt = m_controlPoints[idx].lock())
				deBoorPositions[idx] = { pt->m_position.x, pt->m_position.y, pt->m_position.z };
		}
	}
	device.UpdateBuffer(m_deBoorVertexBuffer, deBoorPositions.data(), deBoorCount * sizeof(VertexPosition));


	// 2. Prepare the augmented 2D grid
	int ptsU = GetDeBoorPointsU();
	int ptsV = GetDeBoorPointsV();

	// A. Extract actual user points into a 2D array
	std::vector<std::vector<Vec3f>> userGrid(ptsU, std::vector<Vec3f>(ptsV, { 0,0,0 }));
	for (int v = 0; v < ptsV; v++)
	{
		for (int u = 0; u < ptsU; u++)
		{
			if (auto pt = GetPoint(u, v))
				userGrid[u][v] = pt->m_position.ToVec3f();
		}
	}

	// B. Create Augmented Grid (Add 2 points for extrapolating flat edges)
	int augU = (shapeType == SurfaceShape::Cylinder) ? ptsU : ptsU + 2;
	int augV = ptsV + 2;
	std::vector<std::vector<Vec3f>> augGrid(augU, std::vector<Vec3f>(augV, { 0,0,0 }));

	// C. Fill Augmented Grid interior
	for (int v = 0; v < augV; v++)
	{
		int readV = std::clamp(v - 1, 0, ptsV - 1);
		for (int u = 0; u < augU; u++)
		{
			if (shapeType == SurfaceShape::Cylinder)
			{
				int readU = u % ptsU; // Cylinders wrap in U
				augGrid[u][v] = userGrid[readU][readV];
			}
			else
			{
				int readU = std::clamp(u - 1, 0, ptsU - 1);
				augGrid[u][v] = userGrid[readU][readV];
			}
		}
	}

	// D. Extrapolate Phantom Edges (P0 * 2 - P1)
	for (int u = 0; u < augU; u++)
	{
		augGrid[u][0] = augGrid[u][1] * 2.0f - augGrid[u][2];
		augGrid[u][augV - 1] = augGrid[u][augV - 2] * 2.0f - augGrid[u][augV - 3];
	}

	if (shapeType == SurfaceShape::Flat)
	{
		for (int v = 0; v < augV; v++)
		{
			augGrid[0][v] = augGrid[1][v] * 2.0f - augGrid[2][v];
			augGrid[augU - 1][v] = augGrid[augU - 2][v] * 2.0f - augGrid[augU - 3][v];
		}
	}
	// 3. Convert and Upload Bernstein Points (Surface Patches)
	UINT bernsteinCount = GetBernsteinPointsU() * GetBernsteinPointsV();
	std::vector<VertexPosition> bernsteinPositions(bernsteinCount, { 0,0,0 });
	int patchesU = GetPatchesU();
	int patchesV = GetPatchesV();
	for (int patchV = 0; patchV < patchesV; patchV++)
	{
		for (int patchU = 0; patchU < patchesU; patchU++)
		{
			ConvertPatchToBernstein(patchU, patchV, augGrid, bernsteinPositions);
		}
	}
	device.UpdateBuffer(m_bernsteinVertexBuffer, bernsteinPositions.data(), bernsteinCount * sizeof(VertexPosition));

	//m_isDirty = false;
}

SurfaceGenerationResult BSplineSurface::CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device)
{
	auto surface = std::make_unique<BSplineSurface>(segU, segV, SurfaceShape::Flat, true);
	std::vector<std::shared_ptr<Point>> generatedPoints;

	int pointsU = segU + 3;
	int pointsV = segV + 3;

	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float uParam = static_cast<float>(u) / (pointsU - 1);
			float3 pos{ uParam * width + center.x, center.y, vParam * length + center.z };

			auto pt = std::make_shared<Point>(pos, true, true);
			pt->isLockedToSurface = true;
			generatedPoints.push_back(pt);
			surface->m_controlPoints.push_back(pt);
		}
	}

	surface->InitGeometry(device);
	return { std::move(surface), std::move(generatedPoints) };
}

SurfaceGenerationResult BSplineSurface::CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device)
{
	auto surface = std::make_unique<BSplineSurface>(segU, segV, SurfaceShape::Cylinder, true);
	std::vector<std::shared_ptr<Point>> generatedPoints;

	int pointsU = segU;
	int pointsV = segV + 3;

	// Scale De Boor points out so the resulting C2 surface passes exactly through 'radius'
	float dTheta = 2.0f * std::numbers::pi_v<float> / segU;
	float R_deBoor = radius * (3.0f / (2.0f + std::cos(dTheta)));

	for (int v = 0; v < pointsV; v++)
	{
		float vParam = static_cast<float>(v) / (pointsV - 1);
		for (int u = 0; u < pointsU; u++)
		{
			float angle = u * dTheta;
			float3 pos = {
				R_deBoor * std::cos(angle) + center.x,
				R_deBoor * std::sin(angle) + center.y + radius,
				vParam * height + center.z
			};

			auto pt = std::make_shared<Point>(pos, true, true);
			pt->isLockedToSurface = true;
			generatedPoints.push_back(pt);
			surface->m_controlPoints.push_back(pt);
		}
	}

	surface->InitGeometry(device);
	return { std::move(surface), std::move(generatedPoints) };
}

std::vector<unsigned int> BSplineSurface::GenerateLineIndices() const
{
	std::vector<unsigned int> indices;
	int pointsU = GetDeBoorPointsU();
	int pointsV = GetDeBoorPointsV();
	int logicalPointsU = (shapeType == SurfaceShape::Cylinder) ? pointsU + 1 : pointsU;

	for (int v = 0; v < pointsV; v++)
	{
		for (int u = 0; u < logicalPointsU - 1; u++)
		{
			indices.push_back(GetDeBoorIndex(u, v));
			indices.push_back(GetDeBoorIndex(u + 1, v));
		}
	}
	for (int u = 0; u < pointsU; u++)
	{
		for (int v = 0; v < pointsV - 1; ++v)
		{
			indices.push_back(GetDeBoorIndex(u, v));
			indices.push_back(GetDeBoorIndex(u, v + 1));
		}
	}
	return indices;
}

std::vector<unsigned int> BSplineSurface::GeneratePatchIndices() const
{
	std::vector<unsigned int> indices;
	int patchesU = GetPatchesU();
	int patchesV = GetPatchesV();
	indices.reserve(static_cast<size_t>(patchesU) * patchesV * 16);

	for (int patchV = 0; patchV < patchesV; patchV++)
	{
		for (int patchU = 0; patchU < patchesU; patchU++)
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



