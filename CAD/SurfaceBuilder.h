#pragma once
#include "SceneObject.h"
#include "Surface.h"
#include "DxDevice.h"

enum class SurfaceType { C0, C2 };
struct SurfaceBuilder
{
	std::vector<VertexPosition> rawPoints;
	std::vector<VertexPosition> bernsteinPoints;
	std::vector<VertexPositionUV> patchPointsUV;
	std::vector<unsigned int> patchIndices;
	std::vector<std::vector<int>> indexGrid;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchIndexBuffer;
	UINT m_patchIndexCount = 0;

	UINT m_patchVertexBufferCapacity = 0;
	UINT m_patchIndexBufferCapacity = 0;

	SurfaceType surfaceType;
	SurfaceShape surfaceShape;
	int segmentsU, segmentsV;
	union { float dim1, radius, width; };
	union { float dim2, length; };

	float3 center = { 0.0f, 0.0f, 0.0f };
	int linesPerSegmentU = 4, linesPerSegmentV = 4;
	int m_uniquePointsCount = 0;

	void UpdateGeometry(const DxDevice& device, SurfaceType newType, SurfaceShape newShape, int segU, int segV, float d1, float d2, const float3& centerPos);
	SurfaceGenerationResult Build(const DxDevice& device) const;
private:
	template<typename T>
	void UpdateVertexBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<T>& data)
	{
		if (data.size() > capacity)
		{
			capacity = std::max(static_cast<UINT>(data.size()), static_cast<UINT>(capacity * 1.5));
			buffer = device.CreateDynamicVertexBuffer<T>(capacity);
		}
		device.UpdateBuffer(buffer, data.data(), static_cast<size_t>(data.size()) * sizeof(T));
	}

	void GenerateC0Flat();
	void GenerateC0Cylinder();
	void GenerateC2Flat();
	void GenerateC2Cylinder();

	std::vector<VertexPositionUV> GenerateC0PatchPointsUV() const;
	std::vector<VertexPositionUV> GenerateC2BernsteinPointsUV() const;

	std::vector<VertexPosition> GenerateC2BernsteinPoints() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	std::vector<unsigned int> GenerateLineIndices() const;
};
