#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Vec4d.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	TEST_CLASS(Vec4dTest)
	{
	public:
		TEST_METHOD(ValuesConstructorTest)
		{
			Vec4d v(1.0, 2.0, 3.0, 4.0);

			Assert::AreEqual(1.0, v.x);
			Assert::AreEqual(2.0, v.y);
			Assert::AreEqual(3.0, v.z);
			Assert::AreEqual(4.0, v.w);

			Assert::AreEqual(1.0, v.d[0]);
			Assert::AreEqual(2.0, v.d[1]);
			Assert::AreEqual(3.0, v.d[2]);
			Assert::AreEqual(4.0, v.d[3]);
		}

		TEST_METHOD(AddOperatorTest)
		{
			Vec4d a(1.0, 2.0, 3.0, 4.0);
			Vec4d b(5.0, 6.0, 7.0, 8.0);
			Vec4d c = a + b;
			Assert::AreEqual(6.0, c.x);
			Assert::AreEqual(8.0, c.y);
			Assert::AreEqual(10.0, c.z);
			Assert::AreEqual(12.0, c.w);
		}

		TEST_METHOD(SubOperatorTest)
		{
			Vec4d a(5.0, 6.0, 7.0, 8.0);
			Vec4d b(1.0, 2.0, 3.0, 4.0);
			Vec4d c = a - b;
			Assert::AreEqual(4.0, c.x);
			Assert::AreEqual(4.0, c.y);
			Assert::AreEqual(4.0, c.z);
			Assert::AreEqual(4.0, c.w);
		}

		TEST_METHOD(ScalarMulOperatorTest)
		{
			Vec4d v(1.0, 2.0, 3.0, 4.0);
			Vec4d r1 = v * 2.0;
			Vec4d r2 = 2.0 * v;
			Assert::AreEqual(2.0, r1.x);
			Assert::AreEqual(4.0, r1.y);
			Assert::AreEqual(6.0, r1.z);
			Assert::AreEqual(8.0, r1.w);
			Assert::AreEqual(2.0, r2.x);
			Assert::AreEqual(4.0, r2.y);
			Assert::AreEqual(6.0, r2.z);
			Assert::AreEqual(8.0, r2.w);
		}

		TEST_METHOD(ScalarDivOperatorTest)
		{
			Vec4d v(2.0, 4.0, 6.0, 8.0);
			Vec4d r = v / 2.0;
			Assert::AreEqual(1.0, r.x);
			Assert::AreEqual(2.0, r.y);
			Assert::AreEqual(3.0, r.z);
			Assert::AreEqual(4.0, r.w);
		}

		TEST_METHOD(DotTest)
		{
			Vec4d a(1.0, 2.0, 3.0, 4.0);
			Vec4d b(5.0, 6.0, 7.0, 8.0);
			double dot = Vec4d::dot(a, b);
			Assert::AreEqual(70.0, dot);
		}

		TEST_METHOD(OrthogonalDotTest)
		{
			Vec4d a(1.0, 0.0, 0.0, 0.0);
			Vec4d b(0.0, 1.0, 0.0, 0.0);
			double dot = Vec4d::dot(a, b);
			Assert::AreEqual(0.0, dot);
		}

		TEST_METHOD(LengthSqrTest)
		{
			Vec4d v(1.0, 2.0, 3.0, 4.0);
			double lenSqr = v.length_sqr();
			Assert::AreEqual(30.0, lenSqr);
		}

		TEST_METHOD(LengthTest)
		{
			Vec4d v(1.0, 2.0, 2.0, 4.0);
			double len = v.length();
			Assert::AreEqual(5.0, len);
		}
	};
}
