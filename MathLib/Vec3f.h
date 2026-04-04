#pragma once
#include "Vec4f.h"
namespace MathLib
{
	struct alignas(16) Vec3f
	{
		union
		{
			__m128 v;
			float f[3];
			struct
			{
				float x, y, z;
			};
		};

		Vec3f() : v(_mm_setzero_ps()) {}
		Vec3f(__m128 val) : v(_mm_blend_ps(val, _mm_setzero_ps(), 0b1000)) {}
		Vec3f(float x, float y, float z) : v(_mm_set_ps(0.0f, z, y, x)) {}

		static Vec3f FromVec4f(const Vec4f& vec4) {
			return _mm_blend_ps(vec4.v, _mm_setzero_ps(), 0b1000);
		}

		Vec4f ToVec4f(float w = 0.0f) const {
			return _mm_blend_ps(v, _mm_set1_ps(w), 0b1000);
		}

		friend Vec3f operator+(const Vec3f& a, const Vec3f& b)
		{
			return _mm_add_ps(a.v, b.v);
		}

		friend Vec3f& operator +=(Vec3f& a, const Vec3f& b)
		{
			a = a + b;
			return a;
		}

		friend Vec3f operator-(const Vec3f& a, const Vec3f& b)
		{
			return _mm_sub_ps(a.v, b.v);
		}

		friend Vec3f operator*(const Vec3f& vec, float scalar)
		{
			return _mm_mul_ps(vec.v, _mm_set1_ps(scalar));
		}

		friend Vec3f operator*(float scalar, const Vec3f& vec)
		{
			return vec * scalar;
		}

		friend Vec3f& operator*=(Vec3f& vec, float scalar)
		{
			vec = vec * scalar;
			return vec;
		}

		friend Vec3f operator/(const Vec3f& vec, float scalar)
		{
			return _mm_div_ps(vec.v, _mm_set1_ps(scalar));
		}

		static float dot(const Vec3f& a, const Vec3f& b)
		{
			// 0x71 mask: 
			// (0x7) multiply and sum elements 0, 1, and 2 (x, y, z).
			// (0x1) store the sum in element 0 and zero out the rest.
			return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0x71));
		}

		float length_sqr() const
		{
			return dot(*this, *this);
		}

		float length() const
		{
			return std::sqrt(length_sqr());
		}

		Vec3f normalize() const
		{
			float len = length();
			if (len < 1e-6f)
				return Vec3f(0.0f, 0.0f, 0.0f);
			return *this / len;
		}

		static Vec3f cross(const Vec3f& a, const Vec3f& b)
		{
			// Layout 1: A = (y, z, x, w), B = (z, x, y, w)
			__m128 a_yzx = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 b_zxy = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 1, 0, 2));

			// Layout 2: A = (z, x, y, w), B = (y, z, x, w)
			__m128 a_zxy = _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(3, 1, 0, 2));
			__m128 b_yzx = _mm_shuffle_ps(b.v, b.v, _MM_SHUFFLE(3, 0, 2, 1));

			// Multiply layouts
			__m128 mul1 = _mm_mul_ps(a_yzx, b_zxy);
			__m128 mul2 = _mm_mul_ps(a_zxy, b_yzx);

			// Subtract to get the final cross product
			return _mm_sub_ps(mul1, mul2);
		}
	};
}