#pragma once
#include "SceneObject.h"
#include "Point.h"

enum class SurfaceShape { Flat, Cylinder };

struct BezierSurface;
struct SurfaceGenerationResult
{
	std::unique_ptr<BezierSurface> surface;
	std::vector<std::shared_ptr<Point>> points;
};

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

