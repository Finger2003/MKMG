#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Vec3d.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	TEST_CLASS(Vec3dTest)
	{
	public:
		TEST_METHOD(ValuesConstructorTest)
		{
			Vec3d v(1.0, 2.0, 3.0);

			Assert::AreEqual(1.0, v.x);
			Assert::AreEqual(2.0, v.y);
			Assert::AreEqual(3.0, v.z);

			Assert::AreEqual(1.0, v.d[0]);
			Assert::AreEqual(2.0, v.d[1]);
			Assert::AreEqual(3.0, v.d[2]);
		}

		TEST_METHOD(AddOperatorTest)
		{
			Vec3d a(1.0, 2.0, 3.0);
			Vec3d b(4.0, 5.0, 6.0);
			Vec3d c = a + b;
			Assert::AreEqual(5.0, c.x);
			Assert::AreEqual(7.0, c.y);
			Assert::AreEqual(9.0, c.z);
		}

		TEST_METHOD(SubOperatorTest)
		{
			Vec3d a(4.0, 5.0, 6.0);
			Vec3d b(1.0, 2.0, 3.0);
			Vec3d c = a - b;
			Assert::AreEqual(3.0, c.x);
			Assert::AreEqual(3.0, c.y);
			Assert::AreEqual(3.0, c.z);
		}

		TEST_METHOD(ScalarMulOperatorTest)
		{
			Vec3d v(1.0, 2.0, 3.0);
			Vec3d r1 = v * 2.0;
			Vec3d r2 = 2.0 * v;
			Assert::AreEqual(2.0, r1.x);
			Assert::AreEqual(4.0, r1.y);
			Assert::AreEqual(6.0, r1.z);
			Assert::AreEqual(2.0, r2.x);
			Assert::AreEqual(4.0, r2.y);
			Assert::AreEqual(6.0, r2.z);
		}

		TEST_METHOD(ScalarDivOperatorTest)
		{
			Vec3d v(2.0, 4.0, 6.0);
			Vec3d r = v / 2.0;
			Assert::AreEqual(1.0, r.x);
			Assert::AreEqual(2.0, r.y);
			Assert::AreEqual(3.0, r.z);
		}

		TEST_METHOD(DotTest)
		{
			Vec3d a(1.0, 2.0, 3.0);
			Vec3d b(4.0, 5.0, 6.0);
			double d = Vec3d::dot(a, b);
			Assert::AreEqual(32.0, d);
		}

		TEST_METHOD(OrthogonalDotTest)
		{
			Vec3d a(1.0, 0.0, 0.0);
			Vec3d b(0.0, 1.0, 0.0);
			double d = Vec3d::dot(a, b);
			Assert::AreEqual(0.0, d);
		}

		TEST_METHOD(LengthSqrTest)
		{
			Vec3d v(1.0, 2.0, 3.0);
			double ls = v.length_sqr();
			Assert::AreEqual(14.0, ls);
		}

		TEST_METHOD(LengthTest)
		{
			Vec3d v(1.0, 2.0, 2.0);
			double l = v.length();
			Assert::AreEqual(3.0, l);
		}
	};
}
