#include "pch.h"
#include "Intersection.h"


Intersection::Intersection()
	: SceneObject("Intersection" + std::to_string(s_nextId++), ObjectType::Intersection)
{
}

void Intersection::InitGeometry(const std::vector<MathLib::Vec3f>& points, const DxDevice& device)
{
	if (points.empty())
		return;

	m_vertexCount = static_cast<UINT>(points.size());
	std::vector<VertexPosition> vertices;
	vertices.reserve(points.size());
	std::transform(points.begin(), points.end(), std::back_inserter(vertices), [](const MathLib::Vec3f& p) {
		return ToVertexPosition(p);
		});

	m_vertexBuffer = device.CreateVertexBuffer(vertices);
}
