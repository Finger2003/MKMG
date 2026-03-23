#pragma once


enum class ObjectType
{
	Point,
	Torus
};

struct SceneObject
{
	std::string name;
	ObjectType type;
	bool selected = false;

	//virtual const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const = 0;
	SceneObject(std::string name, ObjectType type) : name(std::move(name)), type(type) {}
	virtual ~SceneObject() = default;
};

