#pragma once
#include "Vec4f.h"
#include "Vec3f.h"

namespace MathLib
{
	struct Quaternion
	{
		float x, y, z, w;
	};

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
			float cosX = std::sqrt(mat.m[2][0] * mat.m[2][0] + mat.m[2][2] * mat.m[2][2]);

			bool singular = cosX < 1e-6; // Check for Gimbal lock

			float x, y, z;
			if (!singular)
			{
				x = std::atan2(mat.m[2][1], cosX);
				y = std::atan2(-mat.m[2][0], mat.m[2][2]);
				z = std::atan2(-mat.m[0][1], mat.m[1][1]);
			}
			else
			{
				// In Gimbal lock, we can set Z to 0 and calculate X
				x = (mat.m[2][1] > 0) ? (std::numbers::pi_v<float> / 2.0f) : (-std::numbers::pi_v<float> / 2.0f);
				y = std::atan2(mat.m[0][2], mat.m[0][0]);
				z = 0;
			}

			return { x, y, z };
		}

		static Mat4f Frustum(float l, float r, float b, float t, float n, float f)
		{
			return Mat4f(
				Vec4f(2.0f * n / (r - l), 0.0f, (r + l) / (r - l), 0.0f),
				Vec4f(0.0f, 2.0f * n / (t - b), (t + b) / (t - b), 0.0f),
				Vec4f(0.0f, 0.0f, f / (n - f), (f * n) / (n - f)),
				Vec4f(0.0f, 0.0f, -1.0f, 0.0f)
			);
		}

		Quaternion ToQuaternion() const
		{
			float r11 = m[0][0], r12 = m[0][1], r13 = m[0][2];
			float r21 = m[1][0], r22 = m[1][1], r23 = m[1][2];
			float r31 = m[2][0], r32 = m[2][1], r33 = m[2][2];

			float trace = r11 + r22 + r33;
			Quaternion q;

			if (trace > 0.0f)
			{
				float s = 0.5f / std::sqrt(trace + 1.0f);
				q.w = 0.25f / s;
				q.x = (r32 - r23) * s;
				q.y = (r13 - r31) * s;
				q.z = (r21 - r12) * s;
			}
			else if (r11 > r22 && r11 > r33)
			{
				float s = 2.0f * std::sqrt(1.0f + r11 - r22 - r33);
				q.w = (r32 - r23) / s;
				q.x = 0.25f * s;
				q.y = (r12 + r21) / s;
				q.z = (r13 + r31) / s;
			}
			else if (r22 > r33)
			{
				float s = 2.0f * std::sqrt(1.0f + r22 - r11 - r33);
				q.w = (r13 - r31) / s;
				q.x = (r12 + r21) / s;
				q.y = 0.25f * s;
				q.z = (r23 + r32) / s;
			}
			else
			{
				float s = 2.0f * std::sqrt(1.0f + r33 - r11 - r22);
				q.w = (r21 - r12) / s;
				q.x = (r13 + r31) / s;
				q.y = (r23 + r32) / s;
				q.z = 0.25f * s;
			}

			return q;
		}
	};
}
