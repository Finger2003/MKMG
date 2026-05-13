#include "pch.h"
#include "BezierSurface.h"
#include "DxDevice.h"

unsigned int BezierSurface::s_nextId = 0;

BezierSurface::BezierSurface(std::string&& name, int uSeg, int vSeg, SurfaceShape shape)
	: SceneObject(std::move(name), ObjectType::BezierSurface),
	segmentsU(uSeg), segmentsV(vSeg), shapeType(shape)
{}

std::shared_ptr<Point> BezierSurface::GetPoint(int u, int v) const
{
	size_t pointsU = (shapeType == SurfaceShape::Cylinder) ? (static_cast<unsigned long long>(3) * segmentsU) : (static_cast<size_t>(3) * segmentsU + 1);
	size_t wrappedU = u % pointsU;
	return m_controlPoints[v * pointsU + wrappedU].lock();
}

void BezierSurface::UpdatePatches(const DxDevice& device)
{
	if (!m_isDirty)
		return;

	std::vector<VertexPosition> patchVertices;

	for (int patchV = 0; patchV < segmentsV; patchV++)
	{
		for (int patchU = 0; patchU < segmentsU; patchU++)
		{
			for (int v = 0; v < 4; v++)
			{
				for (int u = 0; u < 4; u++)
				{
					if (auto point = GetPoint(patchU * 3 + u, patchV * 3 + v))
						patchVertices.push_back({ point->m_position.x, point->m_position.y, point->m_position.z });
					else
						patchVertices.push_back({ 0.0f, 0.0f, 0.0f });
				}
			}
		}
	}

	UpdateDynamicBuffer(device, m_patchBuffer, m_patchBufferCapacity, patchVertices);
	m_patchVertexCount = static_cast<UINT>(patchVertices.size());
	//m_isDirty = false;
}

void BezierSurface::UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data)
{
	if (data.size() > capacity)
	{
		capacity = std::max({ static_cast<UINT>(data.size()), static_cast<UINT>(capacity * 1.5), 16u });
		buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
	}
	device.UpdateBuffer(buffer, data.data(), static_cast<UINT>(data.size()) * sizeof(VertexPosition));
}

