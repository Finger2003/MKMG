#pragma once
#include "structs.h"

enum class ObjectType
{
	Point,
	Torus
};


struct SceneObject
{
	std::string name;
	float3 m_position;	
	ObjectType type;
	bool selected = false;


	//virtual const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const = 0;
	SceneObject(float3 position, std::string name, ObjectType type) : m_position(position), name(std::move(name)), type(type) {}
	virtual ~SceneObject() = default;
};

