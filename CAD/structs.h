#pragma once
#include "../MathLib/Vec3f.h"
#include "../MathLib/Vec4f.h"
struct VertexPosition
{
	float x, y, z;
};

struct float3
{
	float x, y, z;
	float3() = default;
	float3(float x, float y, float z) : x(x), y(y), z(z) {}
	float3(const MathLib::Vec3f& v) : x(v.x), y(v.y), z(v.z) {}

	MathLib::Vec3f ToVec3f() const { return MathLib::Vec3f(x, y, z); }
	MathLib::Vec4f ToVec4f(float w = 0.0f) const { return MathLib::Vec4f(x, y, z, w); }
	static float3 FromVec4f(const MathLib::Vec4f& v) { return float3(v.x, v.y, v.z); }
};