#pragma once
#include "SceneObject.h"
#include "Point.h"

enum class SurfaceShape { Flat, Cylinder };

struct BezierSurface : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::BezierSurface);
	static unsigned int s_nextId;

	SurfaceShape shapeType;
	int segmentsU;
	int segmentsV;

	std::vector<std::weak_ptr<Point>> m_controlPoints;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchBuffer;
	UINT m_patchVertexCount = 0;
	UINT m_patchBufferCapacity = 0;
	bool m_isDirty = true;

	BezierSurface(int uSeg, int vSeg, SurfaceShape shape, bool isPreview = false);
	void Commit();

	std::shared_ptr<Point> GetPoint(int u, int v) const;
	void UpdatePatches(const DxDevice& device);

protected:
	void UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data);
};