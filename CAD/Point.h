#pragma once
#include "structs.h"
#include "SceneObject.h"
#include "../MathLib/Mat4f.h"

class DxDevice;
struct Point : public TransformableObject
{
	//float3 m_position{ 0.0f, 0.0f, 0.0f };
	static void InitSharedGeometry(const DxDevice& device);
	static void ReleaseSharedGeometry();

	const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const { return s_vertexBuffer; }
	
	Point(float3 position);
	static unsigned int s_nextId;


	MathLib::Mat4f GetModelMatrix() const;
private:
	static Microsoft::WRL::ComPtr<ID3D11Buffer> s_vertexBuffer;
};