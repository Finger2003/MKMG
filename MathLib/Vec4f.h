#pragma once
namespace MathLib
{
	struct alignas(16) Vec4f
	{
		union
		{
			__m128 v;
			float f[4];
			struct
			{
				float x, y, z, w;
			};
		};


		Vec4f() : v(_mm_setzero_ps()) {}
		Vec4f(__m128 val) : v(val) {}
		Vec4f(float x, float y, float z, float w) : v(_mm_set_ps(w, z, y, x)) {}

		friend Vec4f operator+(const Vec4f& a, const Vec4f& b)
		{
			return _mm_add_ps(a.v, b.v);
		}

		friend Vec4f operator-(const Vec4f& a, const Vec4f& b)
		{
			return _mm_sub_ps(a.v, b.v);
		}

		friend Vec4f operator*(const Vec4f& vec, float scalar)
		{
			return _mm_mul_ps(vec.v, _mm_set1_ps(scalar));
		}

		friend Vec4f operator*(float scalar, const Vec4f& vec)
		{
			return vec * scalar;
		}

		friend Vec4f operator/(const Vec4f& vec, float scalar)
		{
			return _mm_div_ps(vec.v, _mm_set1_ps(scalar));
		}

		static float dot(const Vec4f& a, const Vec4f& b)
		{
			return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0xFF));
		}

		float length_sqr() const
		{
			return dot(*this, *this);
		}

		float length() const
		{
			return sqrtf(length_sqr());
		}
	};
}