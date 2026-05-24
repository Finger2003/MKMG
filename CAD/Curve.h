#pragma once
#include "SceneObject.h"
#include "Point.h"

struct Curve : public SceneObject, public IPointDependent
{
	DEFINE_TYPE(SceneObject, ObjectType::Curve);
	std::vector<std::weak_ptr<Point>> m_controlPoints;
	//std::vector<float3> m_lastPositions;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_lineVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_curveVertexBuffer;

	UINT m_lineVertexCount = 0;
	UINT m_curveVertexCount = 0;
	UINT m_lineBufferCapacity = 0;
	UINT m_curveBufferCapacity = 0;
	bool m_isDirty = true;

	void CleanExpiredPoints();
	virtual void UpdatePolyline(const DxDevice& device) = 0;
	nlohmann::json Serialize() const override;
	void MarkDirty() override { m_isDirty = true; }
	void ReplacePoint(Point* oldPoint, std::shared_ptr<Point> newPoint) override;
protected:
	Curve(std::string&& name, ObjectType type, std::vector<std::weak_ptr<Point>>&& controlPoints);
	Curve(unsigned int id, std::string&& name, ObjectType type, std::vector<std::weak_ptr<Point>>&& controlPoints);
	virtual ~Curve() = default;
	void UpdateBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data, UINT& vertexCount, UINT minCount);
};