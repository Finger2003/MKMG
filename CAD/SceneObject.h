#pragma once
#include "structs.h"

//// Generates the base virtual methods returning nullptr
//#define DECLARE_CAST_TYPE(Type) \
//    virtual Type* As##Type() { return nullptr; } \
//    virtual const Type* As##Type() const { return nullptr; }
//
//// Generates the overridden methods returning 'this'
//#define IMPLEMENT_CAST_TYPE(Type) \
//    Type* As##Type() override { return this; } \
//    const Type* As##Type() const override { return this; }


#define SCENE_OBJECT_LIST(X) \
    X(TransformableObject)   \
    X(Point)                 \
    X(Torus)                 \
    X(Curve)                 \
    X(BezierCurve)           \
    X(BSplineCurve)          \
    X(InterpolatingCurve)


enum class ObjectType
{
	SceneObject,
#define AS_ENUM(Name) Name,
	SCENE_OBJECT_LIST(AS_ENUM)
#undef AS_ENUM
};

//enum class ObjectType
//{
//	SceneObject,
//	Transformable,
//	Point,
//	Torus,
//	Curve,
//	BezierCurve,
//	BSplineCurve,
//	InterpolatingCurve
//};

#define DEFINE_TYPE(BaseType, EnumVal) \
    static constexpr ObjectType ClassType =EnumVal; \
    bool IsA(ObjectType t) const override { return t == ClassType || BaseType::IsA(t); }

struct SceneObject
{
	std::string name;
	ObjectType type;
	bool selected = false;

	//virtual const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const = 0;
	SceneObject(std::string&& name, ObjectType type) : name(std::move(name)), type(type) {}
	virtual ~SceneObject() = default;

	static constexpr ObjectType ClassType = ObjectType::SceneObject;
	virtual bool IsA(ObjectType t) const { return t == ClassType; }
	template<typename T> T* As()
	{
		return IsA(T::ClassType) ? static_cast<T*>(this) : nullptr;
	}

	//DECLARE_CAST_TYPE(Curve)
	//DECLARE_CAST_TYPE(TransformableObject)

	//virtual Curve* AsCurve() { return nullptr; }
	//virtual const Curve* AsCurve() const { return nullptr; }

	//virtual TransformableObject* AsTransformable() { return nullptr; }
	//virtual const TransformableObject* AsTransformable() const { return nullptr; }
};

struct TransformableObject : public SceneObject
{
	float3 m_position;
	float3 m_basePosition;

	DEFINE_TYPE(SceneObject, ObjectType::TransformableObject)

	TransformableObject(float3 position, std::string&& name, ObjectType type) 
		: SceneObject(std::move(name), type), m_position(position), m_basePosition(position) {}
	virtual ~TransformableObject() = default;
	
	//IMPLEMENT_CAST_TYPE(TransformableObject)
};

