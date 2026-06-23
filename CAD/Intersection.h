#pragma once
#include "SceneObject.h"
#include "../MathLib/Vec3f.h"
#include "TrimTexture.h"

enum class TrimFillMode
{
	None,
	ClosedLoop,
	BoundaryToBoundary
};

struct Intersection : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::Intersection);

	std::vector<MathLib::Vec4f> m_params; // x = u1, y = v1, z = u2, w = v2
	std::weak_ptr<SceneObject> m_surface1;
	std::weak_ptr<SceneObject> m_surface2;
	MathLib::Vec4f m_maxDomains;

	TrimTexture m_trimTexture1;
	TrimTexture m_trimTexture2;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_uvLinesBuffer1;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_uvLinesBuffer2;
	UINT m_uvLineCount1 = 0;
	UINT m_uvLineCount2 = 0;


	Microsoft::WRL::ComPtr<ID3D11Buffer> m_trimPolygonBuffer1;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_trimPolygonBuffer2;
	UINT m_trimPolygonCount1 = 0;
	UINT m_trimPolygonCount2 = 0;

	TrimFillMode m_trimModeS1 = TrimFillMode::None;
	TrimFillMode m_trimModeS2 = TrimFillMode::None;

	bool m_reverseTrimS1 = false;
	bool m_reverseTrimS2 = false;

	bool m_isClosedLoop = false;
	std::vector<MathLib::Vec3f> m_basePoints;

	Intersection(std::string&& name, std::vector<MathLib::Vec3f>&& basePoints, std::vector<MathLib::Vec4f>&& params,
		std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2,
		MathLib::Vec4f&& maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2, bool isClosedLoop, ObjectType type);

	virtual ~Intersection() = default;

	virtual void InitGeometry(const DxDevice& device);

	std::vector<VertexPosition> GenerateLinesInUVSpace(bool isSurface1);
	std::vector<VertexPosition> GenerateTrimPolygon(bool isSurface1);
};

struct LinearIntersection : public Intersection, public NamedObjectCounter<LinearIntersection>
{
	DEFINE_TYPE(Intersection, ObjectType::LinearIntersection);

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;
	UINT m_vertexCount = 0;

	LinearIntersection(std::vector<MathLib::Vec3f>&& basePoints, std::vector<MathLib::Vec4f>&& params,
		std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2,
		MathLib::Vec4f maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2, bool isClosedLoop);

	void InitGeometry(const DxDevice& device) override;
};

struct BezierIntersection : public Intersection, public NamedObjectCounter<BezierIntersection>
{
	DEFINE_TYPE(Intersection, ObjectType::BezierIntersection);

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_bezierVertexBuffer;
	UINT m_bezierVertexCount = 0;

	//BezierIntersection(std::vector<MathLib::Vec4f> params,
	//	std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2,
	//	MathLib::Vec4f maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2, bool isClosedLoop);

	BezierIntersection(LinearIntersection&& linear);
	void InitGeometry(const DxDevice& device) override;
	void InitBezierGeometry(const DxDevice& device);
};
