#include "pch.h"
#include "Point.h"
#include "DxDevice.h"

using namespace std;
using namespace Microsoft::WRL;

ComPtr<ID3D11Buffer> Point::s_vertexBuffer = nullptr;
unsigned int Point::s_nextId = 0;

void Point::InitSharedGeometry(const DxDevice& device)
{
	if (s_vertexBuffer)
		return; // Already initialized

	vector<VertexPosition> vertices = { {0.0f, 0.0f, 0.0f} };
	s_vertexBuffer = device.CreateVertexBuffer(vertices);
}

void Point::ReleaseSharedGeometry()
{
	s_vertexBuffer.Reset();
}

Point::Point(float3 position, bool lockToSurface)
	: TransformableObject(position,
		"Point" + to_string(s_nextId++),
		ObjectType::Point), isLockedToSurface(lockToSurface)
{}

void Point::AddDependent(std::weak_ptr<SceneObject> obj)
{
	m_dependents.push_back(std::move(obj));
}

void Point::NotifyDependents()
{
	std::erase_if(m_dependents, [](const std::weak_ptr<SceneObject>& weakDep) {
		if (auto dep = weakDep.lock())
		{
			dep->MarkDirty();
			return false;
		}
		return true;
		});
}

MathLib::Mat4f Point::GetModelMatrix() const
{
	return MathLib::Mat4f::Translation(m_position.x, m_position.y, m_position.z);
}