#pragma once
#include "SceneObject.h"
#include "Point.h"

struct BezierCurve : public SceneObject
{
	static unsigned int s_nextId;
	std::vector<std::weak_ptr<Point>> m_controlPoints;

	std::vector<float3> m_lastPositions;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;

	BezierCurve(std::vector<std::weak_ptr<Point>>&& controlPoints);
	void CleanExpiredPoints();
	void UpdatePolyline(const DxDevice& device);
};

