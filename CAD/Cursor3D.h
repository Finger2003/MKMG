#pragma once
#include "structs.h"
#include "../MathLib/Mat4f.h"

class DxDevice;

struct Cursor3D
{
	//float3 position{ 0.0f, 0.0f, 0.0f };

	static void InitSharedGeometry(const DxDevice& device);
	static void ReleaseSharedGeometry();
	static constexpr int VertexCount = 10; // 5 line segments, 2 vertices each

	const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const { return s_vertexBuffer; }

	//MathLib::Mat4f GetTranslationMatrix() const;

private:
	static Microsoft::WRL::ComPtr<ID3D11Buffer> s_vertexBuffer;
};

