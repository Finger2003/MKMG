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
	//static unsigned int s_nextId;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_bernsteinVertexBuffer;
	UINT m_bernsteinVertexCount = 0;
	UINT m_bernsteinBufferCapacity = 0;

	std::vector<VirtualPointMapping> m_virtualPoints;

	BSplineCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	BSplineCurve(unsigned int id, unsigned int bsplineCurveIndex, std::string&& name, std::vector<std::weak_ptr<Point>>&& controlPoints);
	void UpdatePolyline(const DxDevice& device) override;

	const char* GetSchemaType() const override { return "bezierC2"; }
};


