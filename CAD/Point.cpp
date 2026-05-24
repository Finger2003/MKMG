#include "pch.h"
#include "Point.h"
#include "DxDevice.h"

using namespace std;
using namespace Microsoft::WRL;

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

Point::Point(float3 position)
	: TransformableObject(position, "Point" + to_string(s_nextId++), ObjectType::Point)
{}

Point::Point(unsigned int id, float3 position, std::optional<ParsedNameData>&& nameData)
	: TransformableObject(id, position, nameData ? std::move(nameData->name) : "Point" + to_string(s_nextId++), ObjectType::Point)
{
	if (nameData)
		AdvanceCounter(nameData->index);
}

void Point::AddDependent(std::weak_ptr<IPointDependent> obj)
{
	m_dependents.push_back(std::move(obj));
}


void Point::RemoveDependent(IPointDependent* obj)
{
	std::erase_if(m_dependents, [obj](const std::weak_ptr<IPointDependent>& weakDep) {
		if (auto dep = weakDep.lock())
			return dep.get() == obj;
		return true; // Remove expired dependents
		});
}

void Point::NotifyDependents()
{
	std::erase_if(m_dependents, [](const std::weak_ptr<IPointDependent>& weakDep) {
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