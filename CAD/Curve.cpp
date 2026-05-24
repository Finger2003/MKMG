#include "pch.h"
#include "Curve.h"
#include "DxDevice.h"

Curve::Curve(std::string&& name, ObjectType type, std::vector<std::weak_ptr<Point>>&& controlPoints)
    : Base(std::move(name), type), m_controlPoints(std::move(controlPoints))
{}

Curve::Curve(unsigned int id, std::string && name, ObjectType type, std::vector<std::weak_ptr<Point>> && controlPoints)
    : Base(id, std::move(name), type), m_controlPoints(std::move(controlPoints))
{}

void Curve::CleanExpiredPoints()
{
    size_t removedCount = std::erase_if(m_controlPoints, [](const std::weak_ptr<Point>& wp) { return wp.expired(); });
    if (removedCount > 0)
		MarkDirty();
}

nlohmann::json Curve::Serialize() const
{
    nlohmann::json j = Base::Serialize();
    nlohmann::json cpArray = nlohmann::json::array();
    for (const auto& cpWeak : m_controlPoints)
    {
        if (auto cp = cpWeak.lock())
        {
            cpArray.push_back({ {"id", cp->m_id} });
        }
    }
    j["controlPoints"] = cpArray;
    return j;
}

void Curve::ReplacePoint(Point* oldPoint, std::shared_ptr<Point> newPoint)
{
	for (auto& cpWeak : m_controlPoints)
	{
		if (auto cp = cpWeak.lock())
		{
			if (cp.get() == oldPoint)
			{
				cpWeak = newPoint;
			}
		}
	}
	MarkDirty();
}

void Curve::UpdateBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data, UINT& vertexCount, UINT minCount)
{
    if (data.size() >= minCount)
    {
        vertexCount = static_cast<UINT>(data.size());
        if (vertexCount > capacity)
        {
            capacity = std::max({ vertexCount, static_cast<UINT>(capacity * 1.5), 16u });
            buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
        }
        device.UpdateBuffer(buffer, data.data(), vertexCount * sizeof(VertexPosition));
    }
    else
        vertexCount = 0;
}