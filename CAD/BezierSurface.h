#pragma once
#include "SceneObject.h"
#include "Point.h"

enum class SurfaceShape { Flat, Cylinder };

struct BezierSurface;
struct SurfaceGenerationResult
{
	std::unique_ptr<SceneObject> surface;
	std::vector<std::shared_ptr<Point>> points;
};

//struct SurfaceC2GenerationResult
//{
//	std::unique_ptr<BSplineSurface> surface;
//	std::vector<std::shared_ptr<Point>> points;
//};

struct BezierSurface : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::BezierSurface);
	static unsigned int s_nextId;

	SurfaceShape shapeType;
	int segmentsU;
	int segmentsV;

	std::vector<std::weak_ptr<Point>> m_controlPoints;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_polylineIndexBuffer;
	UINT m_polylineIndexCount = 0;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchIndexBuffer;
	UINT m_patchIndexCount = 0;

	bool m_isDirty = true;

	BezierSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview = false);
	void Commit();

	std::shared_ptr<Point> GetPoint(int u, int v) const;
	//void UpdatePatches(const DxDevice& device);
	void InitGeometry(const DxDevice& device);
	void UpdateVertices(const DxDevice& device);
	static SurfaceGenerationResult CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device);
	static SurfaceGenerationResult CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device);
protected:
	std::vector<unsigned int> GenerateLineIndices() const;
	std::vector<unsigned int> GeneratePatchIndices() const;
	unsigned int GetPhysicalPointsU() const;
	unsigned int GetPhysicalPointsV() const;
	unsigned int GetPointIndex(int u, int v) const;
	//void UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data);
};

struct BSplineSurface : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::BSplineSurface);
	static unsigned int s_nextId;

	SurfaceShape shapeType;
	int segmentsU;
	int segmentsV;

	std::vector<std::weak_ptr<Point>> m_controlPoints;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_deBoorVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_polylineIndexBuffer;
	UINT m_polylineIndexCount = 0;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_bernsteinVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchIndexBuffer;
	UINT m_patchIndexCount = 0;

	bool m_isDirty = true;

	BSplineSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview = false);
	void Commit();

	std::shared_ptr<Point> GetPoint(int u, int v) const;
	void InitGeometry(const DxDevice& device);
	void UpdateVertices(const DxDevice& device);

	static SurfaceGenerationResult CreateFlat(int segU, int segV, float width, float length, const float3& center, const DxDevice& device);
	static SurfaceGenerationResult CreateCylinder(int segU, int segV, float radius, float height, const float3& center, const DxDevice& device);
protected:
	std::vector<unsigned int> GenerateLineIndices() const;
	std::vector<unsigned int> GeneratePatchIndices() const;

	unsigned int GetDeBoorPointsU() const;
	unsigned int GetDeBoorPointsV() const;
	unsigned int GetBernsteinPointsU() const;
	unsigned int GetBernsteinPointsV() const;
	unsigned int GetPatchesU() const;
	unsigned int GetPatchesV() const;

	unsigned int GetDeBoorIndex(int u, int v) const;
	unsigned int GetBernsteinIndex(int u, int v) const;

	void ConvertPatchToBernstein(int patchU, int patchV, const std::vector<std::vector<MathLib::Vec3f>>& augGrid, std::vector<VertexPosition>& bernsteinGrid) const;
	MathLib::Vec3f Evaluate1D(MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3, int index) const;
};
