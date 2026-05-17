#pragma once
#include "Surface.h"

struct BSplineSurface : public Surface
{
	DEFINE_TYPE(Surface, ObjectType::BSplineSurface);
	static unsigned int s_nextId;

	//Microsoft::WRL::ComPtr<ID3D11Buffer> m_deBoorVertexBuffer;
	//Microsoft::WRL::ComPtr<ID3D11Buffer> m_bernsteinVertexBuffer;

	BSplineSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview = false);
	void Commit() override;


	void InitGeometry(const DxDevice& device) override;
	void UpdateVertices(const DxDevice& device) override;

	static SurfaceGenerationResult CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device);
	static SurfaceGenerationResult CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device);
protected:
	//std::vector<unsigned int> GenerateLineIndices() const;
	//std::vector<unsigned int> GeneratePatchIndices() const;

	unsigned int GetGridPointsU() const override;
	unsigned int GetGridPointsV() const override;
	unsigned int GetPatchDataIndex(int u, int v) const override;
	//unsigned int GetDeBoorPointsU() const;
	//unsigned int GetDeBoorPointsV() const;
	unsigned int GetBernsteinPointsU() const;
	unsigned int GetBernsteinPointsV() const;

	//unsigned int GetControlPointIndex(int u, int v) const override;
	unsigned int GetBernsteinIndex(int u, int v) const;

	void ConvertPatchToBernstein(int patchU, int patchV, std::vector<VertexPosition>& bernsteinGrid) const;
	MathLib::Vec3f Evaluate1D(MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3, int index) const;
};
