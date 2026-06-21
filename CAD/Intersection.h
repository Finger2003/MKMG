#pragma once
#include "SceneObject.h"
#include "../MathLib/Vec3f.h"
#include "DxDevice.h"

struct Intersection : public SceneObject, public NamedObjectCounter<Intersection>
{
	DEFINE_TYPE(SceneObject, ObjectType::Intersection);

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;
	UINT m_vertexCount = 0;

	Intersection();
	void InitGeometry(const std::vector<MathLib::Vec3f>& points, const DxDevice& device);
};

