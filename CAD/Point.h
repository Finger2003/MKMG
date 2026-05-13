#pragma once
#include "structs.h"
#include "SceneObject.h"
#include "../MathLib/Mat4f.h"

class DxDevice;
struct Point : public TransformableObject
{
	DEFINE_TYPE(TransformableObject, ObjectType::Point);
	static void InitSharedGeometry(const DxDevice& device);
	static void ReleaseSharedGeometry();

	const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const { return s_vertexBuffer; }

	Point(float3 position, bool isPreview = false, bool lockToSurface = false);
	static unsigned int s_nextId;
	bool isLockedToSurface = false;

	void Commit();
	MathLib::Mat4f GetModelMatrix() const;
private:
	static Microsoft::WRL::ComPtr<ID3D11Buffer> s_vertexBuffer;
};