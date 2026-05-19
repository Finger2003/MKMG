#pragma once
#include "structs.h"
//#include <nlohmann/json.hpp>

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
	using Base = BaseType; \
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

	virtual const char* GetSchemaType() const { return nullptr; }
	virtual void MarkDirty() {};
	virtual nlohmann::json Serialize() const = 0;
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


inline nlohmann::json SceneObject::Serialize() const
{
	nlohmann::json j{
		{"id", m_id},
		{"name", name},
	};
	if (const char* schemaType = GetSchemaType())
		j["objectType"] = schemaType;

	return j;
}

struct TransformableObject : public SceneObject
{
	float3 m_position;
	float3 m_basePosition;

	DEFINE_TYPE(SceneObject, ObjectType::TransformableObject);

	nlohmann::json Serialize() const override;

protected:
	TransformableObject(float3 position, std::string&& name, ObjectType type)
		: SceneObject(std::move(name), type), m_position(position), m_basePosition(position)
	{}
	virtual ~TransformableObject() = default;
};

inline nlohmann::json TransformableObject::Serialize() const
{
	nlohmann::json j = SceneObject::Serialize();
	j["position"] = m_position;
	return j;
}