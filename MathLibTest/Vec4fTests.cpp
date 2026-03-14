#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Vec4f.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	TEST_CLASS(Vec4fTest)
	{
	public:
		TEST_METHOD(ValuesConstructorTest)
		{
			Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);

			Assert::AreEqual(1.0f, v.x);
			Assert::AreEqual(2.0f, v.y);
			Assert::AreEqual(3.0f, v.z);
			Assert::AreEqual(4.0f, v.w);

			Assert::AreEqual(1.0f, v.f[0]);
			Assert::AreEqual(2.0f, v.f[1]);
			Assert::AreEqual(3.0f, v.f[2]);
			Assert::AreEqual(4.0f, v.f[3]);
		}

		TEST_METHOD(AddOperatorTest)
		{
			Vec4f a(1.0f, 2.0f, 3.0f, 4.0f);
			Vec4f b(5.0f, 6.0f, 7.0f, 8.0f);
			Vec4f c = a + b;
			Assert::AreEqual(6.0f, c.x);
			Assert::AreEqual(8.0f, c.y);
			Assert::AreEqual(10.0f, c.z);
			Assert::AreEqual(12.0f, c.w);
		}

		TEST_METHOD(SubOperatorTest)
		{
			Vec4f a(5.0f, 6.0f, 7.0f, 8.0f);
			Vec4f b(1.0f, 2.0f, 3.0f, 4.0f);
			Vec4f c = a - b;
			Assert::AreEqual(4.0f, c.x);
			Assert::AreEqual(4.0f, c.y);
			Assert::AreEqual(4.0f, c.z);
			Assert::AreEqual(4.0f, c.w);
		}

		TEST_METHOD(ScalarMulOperatorTest)
		{
			Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
			Vec4f r1 = v * 2.0f;
			Vec4f r2 = 2.0f * v;
			Assert::AreEqual(2.0f, r1.x);
			Assert::AreEqual(4.0f, r1.y);
			Assert::AreEqual(6.0f, r1.z);
			Assert::AreEqual(8.0f, r1.w);
			Assert::AreEqual(2.0f, r2.x);
			Assert::AreEqual(4.0f, r2.y);
			Assert::AreEqual(6.0f, r2.z);
			Assert::AreEqual(8.0f, r2.w);
		}

		TEST_METHOD(ScalarDivOperatorTest)
		{
			Vec4f v(2.0f, 4.0f, 6.0f, 8.0f);
			Vec4f r = v / 2.0f;
			Assert::AreEqual(1.0f, r.x);
			Assert::AreEqual(2.0f, r.y);
			Assert::AreEqual(3.0f, r.z);
			Assert::AreEqual(4.0f, r.w);
		}

		TEST_METHOD(DotTest)
		{
			Vec4f a(1.0f, 2.0f, 3.0f, 4.0f);
			Vec4f b(5.0f, 6.0f, 7.0f, 8.0f);
			float dot = Vec4f::dot(a, b);
			Assert::AreEqual(70.0f, dot);
		}

		TEST_METHOD(OrthogonalDotTest)
		{
			Vec4f a(1.0f, 0.0f, 0.0f, 0.0f);
			Vec4f b(0.0f, 1.0f, 0.0f, 0.0f);
			float dot = Vec4f::dot(a, b);
			Assert::AreEqual(0.0f, dot);
		}

		TEST_METHOD(LengthSqrTest)
		{
			Vec4f v(1.0f, 2.0f, 3.0f, 4.0f);
			float lenSqr = v.length_sqr();
			Assert::AreEqual(30.0f, lenSqr);
		}

		TEST_METHOD(LengthTest)
		{
			Vec4f v(1.0f, 2.0f, 2.0f, 4.0f);
			float len = v.length();
			Assert::AreEqual(5.0f, len);
		}
	};
}
