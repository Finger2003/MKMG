#pragma once
#include "structs.h"
#include "SceneObject.h"
#include "../MathLib/Mat4f.h"

class DxDevice;
struct Point : public TransformableObject, public NamedObjectCounter<Point>
{
	DEFINE_TYPE(TransformableObject, ObjectType::Point);
	static void InitSharedGeometry(const DxDevice& device);
	static void ReleaseSharedGeometry();
	Point(float3 position, bool lockToSurface = false);
	Point(unsigned int id, float3 position, std::optional<ParsedNameData>&& nameData);
	

	static const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetSharedVertexBuffer() { return s_vertexBuffer; }
	const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const { return s_vertexBuffer; }
	std::vector<std::weak_ptr<SceneObject>> m_dependents;
	bool isLockedToSurface = false;

	void AddDependent(std::weak_ptr<SceneObject> obj);
	void RemoveDependent(SceneObject* obj);
	void NotifyDependents();
	MathLib::Mat4f GetModelMatrix() const;
private:
	inline static Microsoft::WRL::ComPtr<ID3D11Buffer> s_vertexBuffer = nullptr;
};