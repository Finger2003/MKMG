#pragma once
#include "Vec4d.h"

namespace MathLib
{
	struct alignas(32) Mat4d
	{
		union
		{
			Vec4d rows[4];
			double m[4][4];
		};
		Mat4d() : rows{ Vec4d(), Vec4d(), Vec4d(), Vec4d() } {}

		Mat4d(const Vec4d& r0, const Vec4d& r1, const Vec4d& r2, const Vec4d& r3) : rows{ r0, r1, r2, r3 } {}

		static Mat4d Identity()
		{
			return Mat4d(
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(0.0, 1.0, 0.0, 0.0),
				Vec4d(0.0, 0.0, 1.0, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		static Mat4d Diagonal(double a, double b, double c, double d)
		{
			return Mat4d(
				Vec4d(a, 0.0, 0.0, 0.0),
				Vec4d(0.0, b, 0.0, 0.0),
				Vec4d(0.0, 0.0, c, 0.0),
				Vec4d(0.0, 0.0, 0.0, d)
			);
		}

		static Mat4d RotationX(double angle)
		{
			double c = std::cos(angle);
			double s = std::sin(angle);
			return Mat4d(
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(0.0, c, -s, 0.0),
				Vec4d(0.0, s, c, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		static Mat4d RotationY(double angle)
		{
			double c = std::cos(angle);
			double s = std::sin(angle);
			return Mat4d(
				Vec4d(c, 0.0, s, 0.0),
				Vec4d(0.0, 1.0, 0.0, 0.0),
				Vec4d(-s, 0.0, c, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		static Mat4d RotationZ(double angle)
		{
			double c = std::cos(angle);
			double s = std::sin(angle);
			return Mat4d(
				Vec4d(c, -s, 0.0, 0.0),
				Vec4d(s, c, 0.0, 0.0),
				Vec4d(0.0, 0.0, 1.0, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		static Mat4d Translation(double tx, double ty, double tz)
		{
			return Mat4d(
				Vec4d(1.0, 0.0, 0.0, tx),
				Vec4d(0.0, 1.0, 0.0, ty),
				Vec4d(0.0, 0.0, 1.0, tz),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		static Mat4d Scaling(double sx, double sy, double sz)
		{
			return Mat4d(
				Vec4d(sx, 0.0, 0.0, 0.0),
				Vec4d(0.0, sy, 0.0, 0.0),
				Vec4d(0.0, 0.0, sz, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);
		}

		friend Vec4d operator*(const Mat4d& mat, const Vec4d& vec)
		{
			return Vec4d(
				Vec4d::dot(mat.rows[0], vec),
				Vec4d::dot(mat.rows[1], vec),
				Vec4d::dot(mat.rows[2], vec),
				Vec4d::dot(mat.rows[3], vec)
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

				__m256d res = _mm256_mul_pd(x, b.rows[0].v);

				res = _mm256_fmadd_pd(y, b.rows[1].v, res);
				res = _mm256_fmadd_pd(z, b.rows[2].v, res);
				res = _mm256_fmadd_pd(w, b.rows[3].v, res);

				result.rows[i].v = res;
			}
			return result;
		}

		Mat4d Transpose() const
		{
			// tmp0 = [a0, b0, a2, b2]
			// tmp1 = [a1, b1, a3, b3]
			__m256d tmp0 = _mm256_unpacklo_pd(rows[0].v, rows[1].v);
			__m256d tmp1 = _mm256_unpackhi_pd(rows[0].v, rows[1].v);

			// tmp2 = [c0, d0, c2, d2]
			// tmp3 = [c1, d1, c3, d3]
			__m256d tmp2 = _mm256_unpacklo_pd(rows[2].v, rows[3].v);
			__m256d tmp3 = _mm256_unpackhi_pd(rows[2].v, rows[3].v);

			Mat4d result;
			// Row 0: [a0, b0, c0, d0]
			result.rows[0].v = _mm256_permute2f128_pd(tmp0, tmp2, 0x20);
			// Row 1: [a1, b1, c1, d1]
			result.rows[1].v = _mm256_permute2f128_pd(tmp1, tmp3, 0x20);
			// Row 2: [a2, b2, c2, d2]
			result.rows[2].v = _mm256_permute2f128_pd(tmp0, tmp2, 0x31);
			// Row 3: [a3, b3, c3, d3]
			result.rows[3].v = _mm256_permute2f128_pd(tmp1, tmp3, 0x31);

			return result;
		}
	};
}