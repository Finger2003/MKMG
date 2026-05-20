#pragma once
#include "SceneObject.h"
#include "Surface.h"

enum class SurfaceType { C0, C2 };
struct SurfaceBuilder
{
	std::vector<VertexPosition> rawPoints;
	std::vector<VertexPosition> bernsteinPoints;
	std::vector<unsigned int> patchIndices;
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

	void UpdateGeometry(const DxDevice& device, SurfaceType newType, SurfaceShape newShape, int segU, int segV, float d1, float d2, const float3& centerPos);
	SurfaceGenerationResult Build(const DxDevice& device) const;
private:
	void UpdateVertexBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data);

	void GenerateC0Flat();
	void GenerateC0Cylinder();
	void GenerateC2Flat();
	void GenerateC2Cylinder();
	std::vector<VertexPosition> GenerateC2BernsteinPoints() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	std::vector<unsigned int> GenerateLineIndices() const;
};
