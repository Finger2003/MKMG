#pragma once
#include "SceneObject.h"
#include "Point.h"
#include "HoleDetection.h"

struct ContinuityVector
{
	float3 start, end;
};

struct GregoryPatch : public SceneObject, public NamedObjectCounter<GregoryPatch>, public IPointDependent
{
	DEFINE_TYPE(SceneObject, ObjectType::GregoryPatch);

	//std::vector<std::weak_ptr<Point>> m_controlPoints;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_continuityVertexBuffer;
	const UINT m_vertexCount = 60;
	const UINT m_continuityVertexCount = 60;


	int m_linesPerSegmentU = 4;
	int m_linesPerSegmentV = 4;
	int m_smoothness = 25;

	Hole3Cycle m_hole;
	bool m_isDirty = true;

	GregoryPatch(Hole3Cycle&& hole);
	virtual ~GregoryPatch() = default;

	void InitGeometry(const DxDevice& device);
	void UpdateVertices(const DxDevice& device);

	void ReplacePoint(Point* oldPoint, std::shared_ptr<Point> newPoint) override;
	void MarkDirty() override { m_isDirty = true; }
};

