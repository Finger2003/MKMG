#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Mat4d.h"
#include "../MathLib/Vec4d.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	TEST_CLASS(Mat4dTest)
	{
	public:
		TEST_METHOD(IdentityTest)
		{
			Mat4d identity = Mat4d::Identity();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					double expected = (i == j) ? 1.0 : 0.0;
					Assert::AreEqual(expected, identity.m[i][j]);
				}
			}
		}

		TEST_METHOD(MatVecMulTest)
		{
			Mat4d mat(
				Vec4d(1.0, 2.0, 3.0, 4.0),
				Vec4d(5.0, 6.0, 7.0, 8.0),
				Vec4d(9.0, 10.0, 11.0, 12.0),
				Vec4d(13.0, 14.0, 15.0, 16.0)
			);
			Vec4d vec(1.0, 1.0, 1.0, 1.0);
			Vec4d result = mat * vec;
			Assert::AreEqual(10.0, result.x);
			Assert::AreEqual(26.0, result.y);
			Assert::AreEqual(42.0, result.z);
			Assert::AreEqual(58.0, result.w);
		}

		TEST_METHOD(IdentityMatVecMulTest)
		{
			Mat4d identity = Mat4d::Identity();
			Vec4d vec(1.0, 2.0, 3.0, 4.0);
			Vec4d result = identity * vec;
			Assert::AreEqual(vec.x, result.x);
			Assert::AreEqual(vec.y, result.y);
			Assert::AreEqual(vec.z, result.z);
			Assert::AreEqual(vec.w, result.w);
		}

		TEST_METHOD(MatMulTest)
		{
			Mat4d a(
				Vec4d(1.0, 2.0, 3.0, 4.0),
				Vec4d(5.0, 6.0, 7.0, 8.0),
				Vec4d(9.0, 10.0, 11.0, 12.0),
				Vec4d(13.0, 14.0, 15.0, 16.0)
			);
			Mat4d b(
				Vec4d(16.0, 15.0, 14.0, 13.0),
				Vec4d(12.0, 11.0, 10.0, 9.0),
				Vec4d(8.0, 7.0, 6.0, 5.0),
				Vec4d(4.0, 3.0, 2.0, 1.0)
			);
			Mat4d result = a * b;

			Assert::AreEqual(80.0, result.m[0][0]);
			Assert::AreEqual(70.0, result.m[0][1]);
			Assert::AreEqual(60.0, result.m[0][2]);
			Assert::AreEqual(50.0, result.m[0][3]);
			Assert::AreEqual(240.0, result.m[1][0]);
			Assert::AreEqual(214.0, result.m[1][1]);
			Assert::AreEqual(188.0, result.m[1][2]);
			Assert::AreEqual(162.0, result.m[1][3]);
			Assert::AreEqual(400.0, result.m[2][0]);
			Assert::AreEqual(358.0, result.m[2][1]);
			Assert::AreEqual(316.0, result.m[2][2]);
			Assert::AreEqual(274.0, result.m[2][3]);
			Assert::AreEqual(560.0, result.m[3][0]);
			Assert::AreEqual(502.0, result.m[3][1]);
			Assert::AreEqual(444.0, result.m[3][2]);
			Assert::AreEqual(386.0, result.m[3][3]);
		}

		TEST_METHOD(IdentityMatMulTest)
		{
			Mat4d identity = Mat4d::Identity();
			Mat4d mat(
				Vec4d(1.0, 2.0, 3.0, 4.0),
				Vec4d(5.0, 6.0, 7.0, 8.0),
				Vec4d(9.0, 10.0, 11.0, 12.0),
				Vec4d(13.0, 14.0, 15.0, 16.0)
			);
			Mat4d result1 = identity * mat;
			Mat4d result2 = mat * identity;
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					Assert::AreEqual(mat.m[i][j], result1.m[i][j]);
					Assert::AreEqual(mat.m[i][j], result2.m[i][j]);
				}
			}
		}
	};
}