#pragma once
#include "structs.h"

enum class ObjectType
{
	Point,
	Torus,
	BezierCurve,
	BSplineCurve,
	InterpolatingCurve
};


struct SceneObject
{
	std::string name;
	ObjectType type;
	bool selected = false;

	//virtual const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const = 0;
	SceneObject(std::string&& name, ObjectType type) : name(std::move(name)), type(type) {}
	virtual ~SceneObject() = default;
};

struct TransformableObject : public SceneObject
{
	float3 m_position;
	float3 m_basePosition;

	TransformableObject(float3 position, std::string&& name, ObjectType type) 
		: SceneObject(std::move(name), type), m_position(position), m_basePosition(position) {}
	virtual ~TransformableObject() = default;
};

