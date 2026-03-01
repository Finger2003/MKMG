#pragma once
#include "Vec4d.h"

namespace MathLib
{
	struct alignas(32) Mat4d
	{
		union
		{
			Vec4d v[4];
			double m[4][4];
		};
		Mat4d() : v{ Vec4d(), Vec4d(), Vec4d(), Vec4d() } {}

		Mat4d(const Vec4d& r0, const Vec4d& r1, const Vec4d& r2, const Vec4d& r3) : v{ r0, r1, r2, r3 } {}

		static Mat4d Identity()
		{
			return Mat4d(
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(0.0, 1.0, 0.0, 0.0),
				Vec4d(0.0, 0.0, 1.0, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		friend Vec4d operator*(const Mat4d& mat, const Vec4d& vec)
		{
			return Vec4d(
				Vec4d::dot(mat.v[0], vec),
				Vec4d::dot(mat.v[1], vec),
				Vec4d::dot(mat.v[2], vec),
				Vec4d::dot(mat.v[3], vec)
			);
		}

		friend Mat4d operator*(const Mat4d& a, const Mat4d& b)
		{
			Mat4d result;
			for (int i = 0; i < 4; ++i)
			{
				__m256d x = _mm256_set1_pd(a.m[i][0]);
				__m256d y = _mm256_set1_pd(a.m[i][1]);
				__m256d z = _mm256_set1_pd(a.m[i][2]);
				__m256d w = _mm256_set1_pd(a.m[i][3]);

				//__m256d r0 = _mm256_mul_pd(x, b.v[0].v);
				//__m256d r1 = _mm256_mul_pd(y, b.v[1].v);
				//__m256d r2 = _mm256_mul_pd(z, b.v[2].v);
				//__m256d r3 = _mm256_mul_pd(w, b.v[3].v);

				//__m256d sum01 = _mm256_add_pd(r0, r1);
				//__m256d sum23 = _mm256_add_pd(r2, r3);

				//result.v[i].v = _mm256_add_pd(sum01, sum23);

				__m256d res = _mm256_mul_pd(x, b.v[0].v);

				res = _mm256_fmadd_pd(y, b.v[1].v, res);
				res = _mm256_fmadd_pd(z, b.v[2].v, res);
				res = _mm256_fmadd_pd(w, b.v[3].v, res);

				result.v[i].v = res;
			}
			return result;
		}
	};
}