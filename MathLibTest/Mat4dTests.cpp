#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Mat4d.h"
#include "../MathLib/Vec4d.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	constexpr double epsilon = 1e-10;
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

		TEST_METHOD(TranspositionTest)
		{
			Mat4d mat(
				Vec4d(1.0, 2.0, 3.0, 4.0),
				Vec4d(5.0, 6.0, 7.0, 8.0),
				Vec4d(9.0, 10.0, 11.0, 12.0),
				Vec4d(13.0, 14.0, 15.0, 16.0)
			);

			Mat4d transposed = mat.Transpose();

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					Assert::AreEqual(mat.m[i][j], transposed.m[j][i]);
				}
			}
		}

		TEST_METHOD(TranslationTest)
		{
			Mat4d translation = Mat4d::Translation(1.0, 2.0, 3.0);
			Vec4d point(1.0, 1.0, 1.0, 1.0);
			Vec4d result = translation * point;
			Assert::AreEqual(2.0, result.x);
			Assert::AreEqual(3.0, result.y);
			Assert::AreEqual(4.0, result.z);
			Assert::AreEqual(1.0, result.w);
		}

		TEST_METHOD(RotationXTest)
		{
			double angle = std::numbers::pi / 2; // 90 degrees
			Mat4d rotation = Mat4d::RotationX(angle);
			Vec4d point(0.0, 1.0, 0.0, 1.0);
			Vec4d result = rotation * point;
			Assert::AreEqual(0.0, result.x, epsilon);
			Assert::AreEqual(0.0, result.y, epsilon);
			Assert::AreEqual(1.0, result.z, epsilon);
			Assert::AreEqual(1.0, result.w, epsilon);
		}

		TEST_METHOD(RotationYTest)
		{
			double angle = std::numbers::pi / 2; // 90 degrees
			Mat4d rotation = Mat4d::RotationY(angle);
			Vec4d point(1.0, 0.0, 0.0, 1.0);
			Vec4d result = rotation * point;
			Assert::AreEqual(0.0, result.x, epsilon);
			Assert::AreEqual(0.0, result.y, epsilon);
			Assert::AreEqual(-1.0, result.z, epsilon);
			Assert::AreEqual(1.0, result.w, epsilon);
		}

		TEST_METHOD(RotationZTest)
		{
			double angle = std::numbers::pi / 2; // 90 degrees
			Mat4d rotation = Mat4d::RotationZ(angle);
			Vec4d point(1.0, 0.0, 0.0, 1.0);
			Vec4d result = rotation * point;
			Assert::AreEqual(0.0, result.x, epsilon);
			Assert::AreEqual(1.0, result.y, epsilon);
			Assert::AreEqual(0.0, result.z, epsilon);
			Assert::AreEqual(1.0, result.w, epsilon);
		}

		TEST_METHOD(ScalingTest)
		{
			Mat4d scaling = Mat4d::Scaling(2.0, 3.0, 4.0);
			Vec4d point(1.0, 1.0, 1.0, 1.0);
			Vec4d result = scaling * point;
			Assert::AreEqual(2.0, result.x);
			Assert::AreEqual(3.0, result.y);
			Assert::AreEqual(4.0, result.z);
			Assert::AreEqual(1.0, result.w);
		}

		TEST_METHOD(InverseSingularMatTest)
		{
			Mat4d mat(
				Vec4d(1.0, 2.0, 3.0, 4.0),
				Vec4d(5.0, 6.0, 7.0, 8.0),
				Vec4d(9.0, 10.0, 11.0, 12.0),
				Vec4d(13.0, 14.0, 15.0, 16.0)
			);
			Mat4d inverse;
			bool success = mat.Inverse(inverse);
			Assert::IsFalse(success);						
		}

		TEST_METHOD(InversePascalMatTest)
		{
			Mat4d mat(
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(1.0, 1.0, 0.0, 0.0),
				Vec4d(1.0, 2.0, 1.0, 0.0),
				Vec4d(1.0, 3.0, 3.0, 1.0)
			);
			Mat4d expectedInverse(
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(-1.0, 1.0, 0.0, 0.0),
				Vec4d(1.0, -2.0, 1.0, 0.0),
				Vec4d(-1.0, 3.0, -3.0, 1.0)
			);

			Mat4d inverse;
			bool success = mat.Inverse(inverse);
			Assert::IsTrue(success);
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					Assert::AreEqual(expectedInverse.m[i][j], inverse.m[i][j], epsilon);
				}
			}
		}

		TEST_METHOD(InverseOrthogonalMatTest)
		{
			Mat4d mat(
				Vec4d(0.0, 1.0, 0.0, 0.0),
				Vec4d(-1.0, 0.0, 0.0, 0.0),
				Vec4d(0.0, 0.0, 1.0, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);

			Mat4d expectedInverse(
				Vec4d(0.0, -1.0, 0.0, 0.0),
				Vec4d(1.0, 0.0, 0.0, 0.0),
				Vec4d(0.0, 0.0, 1.0, 0.0),
				Vec4d(0.0, 0.0, 0.0, 1.0)
			);

			Mat4d inverse;
			bool success = mat.Inverse(inverse);
			Assert::IsTrue(success);
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					Assert::AreEqual(expectedInverse.m[i][j], inverse.m[i][j], epsilon);
				}
			}
		}
	};
}