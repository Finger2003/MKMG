#pragma once
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

		friend Vec3f operator+(const Vec3f& a, const Vec3f& b)
		{
			return _mm_add_ps(a.v, b.v);
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
	};
}