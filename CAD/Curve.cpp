#include "pch.h"
#include "Curve.h"
#include "DxDevice.h"

Curve::Curve(std::string name, ObjectType type, std::vector<std::weak_ptr<Point>>&& controlPoints)
    : SceneObject(std::move(name), type), m_controlPoints(std::move(controlPoints))
{}

void Curve::CleanExpiredPoints()
{
    std::erase_if(m_controlPoints, [](const std::weak_ptr<Point>& wp) { return wp.expired(); });
}

void Curve::UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data)
{
    UINT requiredCount = static_cast<UINT>(data.size());
    if (requiredCount > capacity)
    {
        capacity = std::max({ requiredCount, static_cast<UINT>(capacity * 1.5), 16u });
        buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
    }
    device.UpdateBuffer(buffer, data.data(), requiredCount * sizeof(VertexPosition));
}