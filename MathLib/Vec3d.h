#pragma once
namespace MathLib
{
	struct alignas(32) Vec3d
	{
		union
		{
			__m256d v;
			double d[3];
			struct
			{
				double x, y, z;
			};
		};

		Vec3d() : v(_mm256_setzero_pd()) {}
		Vec3d(__m256d val) : v(_mm256_blend_pd(val, _mm256_setzero_pd(), 0b1000)) {}
		Vec3d(double x, double y, double z) : v(_mm256_set_pd(0.0, z, y, x)) {}

		friend Vec3d operator+(const Vec3d& a, const Vec3d& b)
		{
			return _mm256_add_pd(a.v, b.v);
		}

		friend Vec3d operator-(const Vec3d& a, const Vec3d& b)
		{
			return _mm256_sub_pd(a.v, b.v);
		}

		friend Vec3d operator*(const Vec3d& vec, double scalar)
		{
			return _mm256_mul_pd(vec.v, _mm256_set1_pd(scalar));
		}

		friend Vec3d operator*(double scalar, const Vec3d& vec)
		{
			return vec * scalar;
		}

		friend Vec3d operator/(const Vec3d& vec, double scalar)
		{
			return _mm256_div_pd(vec.v, _mm256_set1_pd(scalar));
		}

		static double dot(const Vec3d& a, const Vec3d& b)
		{
			__m256d mul = _mm256_mul_pd(a.v, b.v);
			__m128d lo = _mm256_castpd256_pd128(mul);
			__m128d hi = _mm256_extractf128_pd(mul, 1);
			__m128d sum128 = _mm_add_pd(lo, hi);
			__m128d sum = _mm_hadd_pd(sum128, sum128);
			return _mm_cvtsd_f64(sum);
		}

		double length_sqr() const
		{
			return dot(*this, *this);
		}

		double length() const
		{
			return std::sqrt(length_sqr());
		}
	};
}