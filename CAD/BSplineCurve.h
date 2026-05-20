#pragma once
#include "Curve.h"
#include "Point.h"

struct VirtualPointMapping
{
	std::weak_ptr<Point> targetPoint;
	float weight;
	float3 virtualPosition;
};


struct BSplineCurve : public Curve, public NamedObjectCounter<BSplineCurve>
{
	DEFINE_TYPE(Curve, ObjectType::BSplineCurve);
	DEFINE_SCHEMA("bezierC2");

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_bernsteinVertexBuffer;
	UINT m_bernsteinVertexCount = 0;
	UINT m_bernsteinBufferCapacity = 0;

	std::vector<VirtualPointMapping> m_virtualPoints;

	BSplineCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	BSplineCurve(unsigned int id, std::vector<std::weak_ptr<Point>>&& controlPoints, std::optional<ParsedNameData>&& nameData);
	void UpdatePolyline(const DxDevice& device) override;
};


