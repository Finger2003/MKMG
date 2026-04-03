#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Vec3f.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	TEST_CLASS(Vec3fTest)
	{
	public:
		TEST_METHOD(ValuesConstructorTest)
		{
			Vec3f v(1.0f, 2.0f, 3.0f);

			Assert::AreEqual(1.0f, v.x);
			Assert::AreEqual(2.0f, v.y);
			Assert::AreEqual(3.0f, v.z);

			Assert::AreEqual(1.0f, v.f[0]);
			Assert::AreEqual(2.0f, v.f[1]);
			Assert::AreEqual(3.0f, v.f[2]);
		}

		TEST_METHOD(AddOperatorTest)
		{
			Vec3f a(1.0f, 2.0f, 3.0f);
			Vec3f b(4.0f, 5.0f, 6.0f);
			Vec3f c = a + b;
			Assert::AreEqual(5.0f, c.x);
			Assert::AreEqual(7.0f, c.y);
			Assert::AreEqual(9.0f, c.z);
		}

		TEST_METHOD(SubOperatorTest)
		{
			Vec3f a(4.0f, 5.0f, 6.0f);
			Vec3f b(1.0f, 2.0f, 3.0f);
			Vec3f c = a - b;
			Assert::AreEqual(3.0f, c.x);
			Assert::AreEqual(3.0f, c.y);
			Assert::AreEqual(3.0f, c.z);
		}

		TEST_METHOD(ScalarMulOperatorTest)
		{
			Vec3f v(1.0f, 2.0f, 3.0f);
			Vec3f r1 = v * 2.0f;
			Vec3f r2 = 2.0f * v;
			Assert::AreEqual(2.0f, r1.x);
			Assert::AreEqual(4.0f, r1.y);
			Assert::AreEqual(6.0f, r1.z);
			Assert::AreEqual(2.0f, r2.x);
			Assert::AreEqual(4.0f, r2.y);
			Assert::AreEqual(6.0f, r2.z);
		}

		TEST_METHOD(ScalarDivOperatorTest)
		{
			Vec3f v(2.0f, 4.0f, 6.0f);
			Vec3f r = v / 2.0f;
			Assert::AreEqual(1.0f, r.x);
			Assert::AreEqual(2.0f, r.y);
			Assert::AreEqual(3.0f, r.z);
		}

		TEST_METHOD(DotTest)
		{
			Vec3f a(1.0f, 2.0f, 3.0f);
			Vec3f b(4.0f, 5.0f, 6.0f);
			float d = Vec3f::dot(a, b);
			Assert::AreEqual(32.0f, d);
		}

		TEST_METHOD(OrthogonalDotTest)
		{
			Vec3f a(1.0f, 0.0f, 0.0f);
			Vec3f b(0.0f, 1.0f, 0.0f);
			float d = Vec3f::dot(a, b);
			Assert::AreEqual(0.0f, d);
		}

		TEST_METHOD(LengthSqrTest)
		{
			Vec3f v(1.0f, 2.0f, 3.0f);
			float ls = v.length_sqr();
			Assert::AreEqual(14.0f, ls);
		}

		TEST_METHOD(LengthTest)
		{
			Vec3f v(1.0f, 2.0f, 2.0f);
			float l = v.length();
			Assert::AreEqual(3.0f, l);
		}

		TEST_METHOD(FromVec4f)
		{
			Vec4f v4(1.0f, 2.0f, 3.0f, 4.0f);
			Vec3f v3 = Vec3f::FromVec4f(v4);
			Assert::AreEqual(1.0f, v3.x);
			Assert::AreEqual(2.0f, v3.y);
			Assert::AreEqual(3.0f, v3.z);
		}
	};
}
