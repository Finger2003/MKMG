#pragma once

namespace MathLib
{
	struct Vec2f
	{
		union
		{
			float f[2];
			struct
			{
				float x, y;
			};
		};

		Vec2f() : x(0.0f), y(0.0f) {}
		Vec2f(float x, float y) : x(x), y(y) {}
		Vec2f(float val) : x(val), y(val) {}

		friend Vec2f operator+(const Vec2f& a, const Vec2f& b)
		{
			return Vec2f(a.x + b.x, a.y + b.y);
		}

		friend Vec2f& operator+=(Vec2f& a, const Vec2f& b)
		{
			a.x += b.x;
			a.y += b.y;
			return a;
		}

		friend Vec2f operator-(const Vec2f& a, const Vec2f& b)
		{
			return Vec2f(a.x - b.x, a.y - b.y);
		}

		friend Vec2f& operator-=(Vec2f& a, const Vec2f& b)
		{
			a.x -= b.x;
			a.y -= b.y;
			return a;
		}

		friend Vec2f operator-(const Vec2f& a)
		{
			return Vec2f(-a.x, -a.y);
		}

		friend Vec2f operator*(const Vec2f& vec, float scalar)
		{
			return Vec2f(vec.x * scalar, vec.y * scalar);
		}

		friend Vec2f operator*(float scalar, const Vec2f& vec)
		{
			return Vec2f(vec.x * scalar, vec.y * scalar);
		}

		friend Vec2f& operator*=(Vec2f& vec, float scalar)
		{
			vec.x *= scalar;
			vec.y *= scalar;
			return vec;
		}

		friend Vec2f operator/(const Vec2f& vec, float scalar)
		{
			float inv = 1.0f / scalar;
			return Vec2f(vec.x * inv, vec.y * inv);
		}

		friend Vec2f& operator /=(Vec2f& vec, float scalar)
		{
			float inv = 1.0f / scalar;
			vec.x *= inv;
			vec.y *= inv;
			return vec;
		}

		static float dot(const Vec2f& a, const Vec2f& b)
		{
			return a.x * b.x + a.y * b.y;
		}

		float length_sqr() const
		{
			return dot(*this, *this);
		}

		float length() const
		{
			return std::sqrt(length_sqr());
		}
	};
}