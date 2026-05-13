#pragma once
#include "structs.h"

#define SCENE_OBJECT_LIST(X) \
    X(TransformableObject)   \
    X(Point)                 \
    X(Torus)                 \
    X(Curve)                 \
    X(BezierCurve)           \
    X(BSplineCurve)          \
    X(InterpolatingCurve)	 \
	X(BezierSurface)


enum class ObjectType
{
	SceneObject,
#define AS_ENUM(Name) Name,
	SCENE_OBJECT_LIST(AS_ENUM)
#undef AS_ENUM
};

#define DEFINE_TYPE(BaseType, EnumVal) \
    static constexpr ObjectType ClassType = EnumVal; \
    bool IsA(ObjectType t) const override { return t == ClassType || BaseType::IsA(t); }

struct SceneObject
{
	std::string name;
	ObjectType type;
	bool selected = false;


	static constexpr ObjectType ClassType = ObjectType::SceneObject;
	virtual bool IsA(ObjectType t) const { return t == ClassType; }
	template<typename T> T* As()
	{
		return IsA(T::ClassType) ? static_cast<T*>(this) : nullptr;
	}
protected:
	virtual ~SceneObject() = default;
	SceneObject(std::string&& name, ObjectType type) : name(std::move(name)), type(type) {}
};

struct TransformableObject : public SceneObject
{
	float3 m_position;
	float3 m_basePosition;

	DEFINE_TYPE(SceneObject, ObjectType::TransformableObject);

protected:
	TransformableObject(float3 position, std::string&& name, ObjectType type)
		: SceneObject(std::move(name), type), m_position(position), m_basePosition(position)
	{}
	virtual ~TransformableObject() = default;
};

