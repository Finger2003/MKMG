#pragma once
#include "Curve.h"
#include "Point.h"

struct BezierCurve : public Curve
{
	static unsigned int s_nextId;
	//std::vector<std::weak_ptr<Point>> m_controlPoints;

	//std::vector<float3> m_lastPositions;
	//Microsoft::WRL::ComPtr<ID3D11Buffer> m_lineVertexBuffer;
	//Microsoft::WRL::ComPtr<ID3D11Buffer> m_curveVertexBuffer;

	//UINT m_lineVertexCount = 0;
	//UINT m_curveVertexCount = 0;

	//UINT m_lineBufferCapacity = 0;
	//UINT m_curveBufferCapacity = 0;

	BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	//void CleanExpiredPoints();
	void UpdatePolyline(const DxDevice& device) override;
	//void UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data);
};

