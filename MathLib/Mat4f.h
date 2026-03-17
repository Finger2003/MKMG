#pragma once
#include "Vec4f.h"
#include "Vec3f.h"

namespace MathLib
{
	struct Mat4f
	{
		union
		{
			Vec4f rows[4];
			float m[4][4];
		};

		Mat4f() : rows{ Vec4f(), Vec4f(), Vec4f(), Vec4f() } {}
		Mat4f(const Vec4f& r0, const Vec4f& r1, const Vec4f& r2, const Vec4f& r3) : rows{ r0, r1, r2, r3 } {}

		static Mat4f Identity()
		{
			return Mat4f(
				Vec4f(1.0f, 0.0f, 0.0f, 0.0f),
				Vec4f(0.0f, 1.0f, 0.0f, 0.0f),
				Vec4f(0.0f, 0.0f, 1.0f, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Mat4f Diagonal(float a, float b, float c, float d)
		{
			return Mat4f(
				Vec4f(a, 0.0f, 0.0f, 0.0f),
				Vec4f(0.0f, b, 0.0f, 0.0f),
				Vec4f(0.0f, 0.0f, c, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, d)
			);
		}

		static Mat4f RotationX(float angle)
		{
			float c = std::cos(angle);
			float s = std::sin(angle);
			return Mat4f(
				Vec4f(1.0f, 0.0f, 0.0f, 0.0f),
				Vec4f(0.0f, c, -s, 0.0f),
				Vec4f(0.0f, s, c, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Mat4f RotationY(float angle)
		{
			float c = std::cos(angle);
			float s = std::sin(angle);
			return Mat4f(
				Vec4f(c, 0.0f, s, 0.0f),
				Vec4f(0.0f, 1.0f, 0.0f, 0.0f),
				Vec4f(-s, 0.0f, c, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Mat4f RotationZ(float angle)
		{
			float c = std::cos(angle);
			float s = std::sin(angle);
			return Mat4f(
				Vec4f(c, -s, 0.0f, 0.0f),
				Vec4f(s, c, 0.0f, 0.0f),
				Vec4f(0.0f, 0.0f, 1.0f, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Mat4f Translation(float tx, float ty, float tz)
		{
			return Mat4f(
				Vec4f(1.0f, 0.0f, 0.0f, tx),
				Vec4f(0.0f, 1.0f, 0.0f, ty),
				Vec4f(0.0f, 0.0f, 1.0f, tz),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Mat4f Scaling(float s)
		{
			return Scaling(s, s, s);
		}

		static Mat4f Scaling(float sx, float sy, float sz)
		{
			return Mat4f(
				Vec4f(sx, 0.0f, 0.0f, 0.0f),
				Vec4f(0.0f, sy, 0.0f, 0.0f),
				Vec4f(0.0f, 0.0f, sz, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		friend Vec4f operator*(const Mat4f& mat, const Vec4f& vec)
		{
			return Vec4f(
				Vec4f::dot(mat.rows[0], vec),
				Vec4f::dot(mat.rows[1], vec),
				Vec4f::dot(mat.rows[2], vec),
				Vec4f::dot(mat.rows[3], vec)
			);
		}

		friend Mat4f operator*(const Mat4f& a, const Mat4f& b)
		{
			Mat4f result;
			for (int i = 0; i < 4; ++i)
			{
				__m128 x = _mm_set1_ps(a.m[i][0]);
				__m128 y = _mm_set1_ps(a.m[i][1]);
				__m128 z = _mm_set1_ps(a.m[i][2]);
				__m128 w = _mm_set1_ps(a.m[i][3]);

				__m128 res = _mm_mul_ps(x, b.rows[0].v);
				res = _mm_fmadd_ps(y, b.rows[1].v, res);
				res = _mm_fmadd_ps(z, b.rows[2].v, res);
				res = _mm_fmadd_ps(w, b.rows[3].v, res);

				result.rows[i].v = res;
			}
			return result;
		}

		Mat4f Transpose() const
		{
			__m128 row0 = rows[0].v;
			__m128 row1 = rows[1].v;
			__m128 row2 = rows[2].v;
			__m128 row3 = rows[3].v;

			_MM_TRANSPOSE4_PS(row0, row1, row2, row3);

			return Mat4f(
				Vec4f(row0),
				Vec4f(row1),
				Vec4f(row2),
				Vec4f(row3)
			);
		}

		static Mat4f Perspective(float fovY, float aspect, float nearZ, float farZ)
		{
			float f = 1.0 / std::tan(fovY / 2.0);
			return Mat4f(
				Vec4f(f / aspect, 0.0, 0.0, 0.0),
				Vec4f(0.0, f, 0.0, 0.0),
				Vec4f(0.0, 0.0, farZ / (nearZ - farZ), (farZ * nearZ) / (nearZ - farZ)),
				Vec4f(0.0, 0.0, -1.0, 0.0)
			);
		}

		static Mat4f RotationAxis(const Vec3f& axis, float angle)
		{
			float c = std::cos(angle);
			float s = std::sin(angle);
			float t = 1.0f - c;

			float x = axis.x;
			float y = axis.y;
			float z = axis.z;

			return Mat4f(
				Vec4f(t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0f),
				Vec4f(t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0f),
				Vec4f(t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0f),
				Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
			);
		}

		static Vec3f ExtractEulerAngles(const MathLib::Mat4f& mat)
		{
			float sy = std::sqrt(mat.m[0][0] * mat.m[0][0] + mat.m[1][0] * mat.m[1][0]);

			bool singular = sy < 1e-6; // Check for Gimbal lock

			float x, y, z;
			if (!singular)
			{
				x = std::atan2(mat.m[2][1], mat.m[2][2]);
				y = std::atan2(-mat.m[2][0], sy);
				z = std::atan2(mat.m[1][0], mat.m[0][0]);
			}
			else
			{
				// In Gimbal lock, we can set Z to 0 and calculate X
				x = std::atan2(-mat.m[1][2], mat.m[1][1]);
				y = std::atan2(-mat.m[2][0], sy);
				z = 0;
			}

			return { x, y, z };
		}
	};
}
