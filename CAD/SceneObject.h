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
	X(Surface)				 \
	X(BezierSurface)		 \
	X(BSplineSurface)


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
	uint32_t m_id = 0;
	bool selected = false;



	static constexpr ObjectType ClassType = ObjectType::SceneObject;
	virtual bool IsA(ObjectType t) const { return t == ClassType; }
	template<typename T> T* As()
	{
		return IsA(T::ClassType) ? static_cast<T*>(this) : nullptr;
	}

	virtual void MarkDirty() {};
	SceneObject(std::string&& name, ObjectType type) : name(std::move(name)), type(type) {}
	virtual ~SceneObject() = default;
protected:
	void AssignGlobalID()
	{
		if (m_id == 0)
		{
			static uint32_t s_globalIdCounter = 1;
			m_id = s_globalIdCounter++;
		}
	}
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

