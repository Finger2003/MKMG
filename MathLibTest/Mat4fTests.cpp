#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/Mat4f.h"
#include "../MathLib/Vec4f.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	constexpr float epsilon = 1e-7f;
	TEST_CLASS(Mat4fTest)
	{
	public:
		TEST_METHOD(IdentityTest)
		{
			Mat4f identity = Mat4f::Identity();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					float expected = (i == j) ? 1.0f : 0.0f;
					Assert::AreEqual(expected, identity.m[i][j]);
				}
			}
		}

		TEST_METHOD(MatVecMulTest)
		{
			Mat4f mat(
				Vec4f(1.0f, 2.0f, 3.0f, 4.0f),
				Vec4f(5.0f, 6.0f, 7.0f, 8.0f),
				Vec4f(9.0f, 10.0f, 11.0f, 12.0f),
				Vec4f(13.0f, 14.0f, 15.0f, 16.0f)
			);
			Vec4f vec(1.0f, 1.0f, 1.0f, 1.0f);
			Vec4f result = mat * vec;
			Assert::AreEqual(10.0f, result.x);
			Assert::AreEqual(26.0f, result.y);
			Assert::AreEqual(42.0f, result.z);
			Assert::AreEqual(58.0f, result.w);
		}

		TEST_METHOD(IdentityMatVecMulTest)
		{
			Mat4f identity = Mat4f::Identity();
			Vec4f vec(1.0f, 2.0f, 3.0f, 4.0f);
			Vec4f result = identity * vec;
			Assert::AreEqual(vec.x, result.x);
			Assert::AreEqual(vec.y, result.y);
			Assert::AreEqual(vec.z, result.z);
			Assert::AreEqual(vec.w, result.w);
		}

		TEST_METHOD(MatMulTest)
		{
			Mat4f a(
				Vec4f(1.0f, 2.0f, 3.0f, 4.0f),
				Vec4f(5.0f, 6.0f, 7.0f, 8.0f),
				Vec4f(9.0f, 10.0f, 11.0f, 12.0f),
				Vec4f(13.0f, 14.0f, 15.0f, 16.0f)
			);
			Mat4f b(
				Vec4f(16.0f, 15.0f, 14.0f, 13.0f),
				Vec4f(12.0f, 11.0f, 10.0f, 9.0f),
				Vec4f(8.0f, 7.0f, 6.0f, 5.0f),
				Vec4f(4.0f, 3.0f, 2.0f, 1.0f)
			);
			Mat4f result = a * b;

			Assert::AreEqual(80.0f, result.m[0][0]);
			Assert::AreEqual(70.0f, result.m[0][1]);
			Assert::AreEqual(60.0f, result.m[0][2]);
			Assert::AreEqual(50.0f, result.m[0][3]);
			Assert::AreEqual(240.0f, result.m[1][0]);
			Assert::AreEqual(214.0f, result.m[1][1]);
			Assert::AreEqual(188.0f, result.m[1][2]);
			Assert::AreEqual(162.0f, result.m[1][3]);
			Assert::AreEqual(400.0f, result.m[2][0]);
			Assert::AreEqual(358.0f, result.m[2][1]);
			Assert::AreEqual(316.0f, result.m[2][2]);
			Assert::AreEqual(274.0f, result.m[2][3]);
			Assert::AreEqual(560.0f, result.m[3][0]);
			Assert::AreEqual(502.0f, result.m[3][1]);
			Assert::AreEqual(444.0f, result.m[3][2]);
			Assert::AreEqual(386.0f, result.m[3][3]);
		}

		TEST_METHOD(IdentityMatMulTest)
		{
			Mat4f identity = Mat4f::Identity();
			Mat4f mat(
				Vec4f(1.0f, 2.0f, 3.0f, 4.0f),
				Vec4f(5.0f, 6.0f, 7.0f, 8.0f),
				Vec4f(9.0f, 10.0f, 11.0f, 12.0f),
				Vec4f(13.0f, 14.0f, 15.0f, 16.0f)
			);
			Mat4f result1 = identity * mat;
			Mat4f result2 = mat * identity;
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
			Mat4f mat(
				Vec4f(1.0f, 2.0f, 3.0f, 4.0f),
				Vec4f(5.0f, 6.0f, 7.0f, 8.0f),
				Vec4f(9.0f, 10.0f, 11.0f, 12.0f),
				Vec4f(13.0f, 14.0f, 15.0f, 16.0f)
			);

			Mat4f transposed = mat.Transpose();

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
			Mat4f translation = Mat4f::Translation(1.0f, 2.0f, 3.0f);
			Vec4f point(1.0f, 1.0f, 1.0f, 1.0f);
			Vec4f result = translation * point;
			Assert::AreEqual(2.0f, result.x);
			Assert::AreEqual(3.0f, result.y);
			Assert::AreEqual(4.0f, result.z);
			Assert::AreEqual(1.0f, result.w);
		}

		TEST_METHOD(RotationXTest)
		{
			float angle = std::numbers::pi_v<float> / 2; // 90 degrees
			Mat4f rotation = Mat4f::RotationX(angle);
			Vec4f point(0.0f, 1.0f, 0.0f, 1.0f);
			Vec4f result = rotation * point;
			Assert::AreEqual(0.0f, result.x, epsilon);
			Assert::AreEqual(0.0f, result.y, epsilon);
			Assert::AreEqual(1.0f, result.z, epsilon);
			Assert::AreEqual(1.0f, result.w, epsilon);
		}

		TEST_METHOD(RotationYTest)
		{
			float angle = std::numbers::pi_v<float> / 2; // 90 degrees
			Mat4f rotation = Mat4f::RotationY(angle);
			Vec4f point(1.0f, 0.0f, 0.0f, 1.0f);
			Vec4f result = rotation * point;
			Assert::AreEqual(0.0f, result.x, epsilon);
			Assert::AreEqual(0.0f, result.y, epsilon);
			Assert::AreEqual(-1.0f, result.z, epsilon);
			Assert::AreEqual(1.0f, result.w, epsilon);
		}

		TEST_METHOD(RotationZTest)
		{
			float angle = std::numbers::pi_v<float> / 2; // 90 degrees
			Mat4f rotation = Mat4f::RotationZ(angle);
			Vec4f point(1.0f, 0.0f, 0.0f, 1.0f);
			Vec4f result = rotation * point;
			Assert::AreEqual(0.0f, result.x, epsilon);
			Assert::AreEqual(1.0f, result.y, epsilon);
			Assert::AreEqual(0.0f, result.z, epsilon);
			Assert::AreEqual(1.0f, result.w, epsilon);
		}

		TEST_METHOD(ScalingTest)
		{
			Mat4f scaling = Mat4f::Scaling(2.0f, 3.0f, 4.0f);
			Vec4f point(1.0f, 1.0f, 1.0f, 1.0f);
			Vec4f result = scaling * point;
			Assert::AreEqual(2.0f, result.x);
			Assert::AreEqual(3.0f, result.y);
			Assert::AreEqual(4.0f, result.z);
			Assert::AreEqual(1.0f, result.w);
		}

		void AssertVec3Equal(const Vec3f& expected, const Vec3f& actual, float epsilon = 1e-5f)
		{
			Assert::AreEqual(expected.x, actual.x, epsilon, L"X-axis mismatch");
			Assert::AreEqual(expected.y, actual.y, epsilon, L"Y-axis mismatch");
			Assert::AreEqual(expected.z, actual.z, epsilon, L"Z-axis mismatch");
		}

		TEST_METHOD(ExtractEulerAnglesIdentityTest)
		{
			MathLib::Mat4f identity = MathLib::Mat4f::Identity(); // Assuming an Identity helper exists
			Vec3f result = Mat4f::ExtractEulerAngles(identity);

			AssertVec3Equal({ 0.0f, 0.0f, 0.0f }, result);
		}

		TEST_METHOD(ExtractEulerAnglesXRotationTest)
		{
			float angle = std::numbers::pi_v<float> / 4; // 45 degrees
			Mat4f rotationX = Mat4f::RotationX(angle);
			Vec3f result = Mat4f::ExtractEulerAngles(rotationX);
			AssertVec3Equal({ angle, 0.0f, 0.0f }, result);
		}

		TEST_METHOD(ExtractEulerAnglesYRotationTest)
		{
			float angle = std::numbers::pi_v<float> / 4; // 45 degrees
			Mat4f rotationY = Mat4f::RotationY(angle);
			Vec3f result = Mat4f::ExtractEulerAngles(rotationY);
			AssertVec3Equal({ 0.0f, angle, 0.0f }, result);
		}

		TEST_METHOD(ExtractEulerAnglesZRotationTest)
		{
			float angle = std::numbers::pi_v<float> / 4; // 45 degrees
			Mat4f rotationZ = Mat4f::RotationZ(angle);
			Vec3f result = Mat4f::ExtractEulerAngles(rotationZ);
			AssertVec3Equal({ 0.0f, 0.0f, angle }, result);
		}

		TEST_METHOD(ExtractEulerAnglesGimbalLockPositive)
		{
			MathLib::Mat4f mat = MathLib::Mat4f::Identity();
			mat.m[2][1] = 1.0f;  // sin(90)
			mat.m[1][1] = 0.0f;  // cos(90)
			mat.m[2][2] = 0.0f;  // cos(90)
			mat.m[1][2] = -1.0f; // -sin(90)

			Vec3f result = Mat4f::ExtractEulerAngles(mat);

			Assert::AreEqual(std::numbers::pi_v<float> / 2.0f, result.x, 1e-6f);
			Assert::AreEqual(0.0f, result.z, 1e-6f);
		}

		TEST_METHOD(ExtractEulerAnglesGimbalLockNegative)
		{
			MathLib::Mat4f mat = MathLib::Mat4f::Identity();
			mat.m[2][1] = -1.0f; // sin(-90)
			mat.m[1][1] = 0.0f;
			mat.m[2][2] = 0.0f;
			mat.m[1][2] = 1.0f;

			Vec3f result = Mat4f::ExtractEulerAngles(mat);

			Assert::AreEqual(-std::numbers::pi_v<float> / 2.0f, result.x, 1e-6f);
			Assert::AreEqual(0.0f, result.z, 1e-6f);
		}

		TEST_METHOD(ExtractEulerAnglesCombinedRotation)
		{
			float x = 0.3f, y = 0.5f, z = -0.2f;
			MathLib::Mat4f mat = MathLib::Mat4f::RotationZ(z) * MathLib::Mat4f::RotationX(x) * MathLib::Mat4f::RotationY(y);

			Vec3f result = Mat4f::ExtractEulerAngles(mat);
			AssertVec3Equal({ x, y, z }, result);
		}
		//TEST_METHOD(InverseSingularMatTest)
		//{
		//	Mat4f mat(
		//		Vec4f(1.0f, 2.0f, 3.0f, 4.0f),
		//		Vec4f(5.0f, 6.0f, 7.0f, 8.0f),
		//		Vec4f(9.0f, 10.0f, 11.0f, 12.0f),
		//		Vec4f(13.0f, 14.0f, 15.0f, 16.0f)
		//	);
		//	Mat4f inverse;
		//	bool success = mat.Inverse(inverse);
		//	Assert::IsFalse(success);
		//}

		//TEST_METHOD(InversePascalMatTest)
		//{
		//	Mat4f mat(
		//		Vec4f(1.0f, 0.0f, 0.0f, 0.0f),
		//		Vec4f(1.0f, 1.0f, 0.0f, 0.0f),
		//		Vec4f(1.0f, 2.0f, 1.0f, 0.0f),
		//		Vec4f(1.0f, 3.0f, 3.0f, 1.0f)
		//	);
		//	Mat4f expectedInverse(
		//		Vec4f(1.0f, 0.0f, 0.0f, 0.0f),
		//		Vec4f(-1.0f, 1.0f, 0.0f, 0.0f),
		//		Vec4f(1.0f, -2.0f, 1.0f, 0.0f),
		//		Vec4f(-1.0f, 3.0f, -3.0f, 1.0f)
		//	);

		//	Mat4f inverse;
		//	bool success = mat.Inverse(inverse);
		//	Assert::IsTrue(success);
		//	for (int i = 0; i < 4; i++)
		//	{
		//		for (int j = 0; j < 4; j++)
		//		{
		//			Assert::AreEqual(expectedInverse.m[i][j], inverse.m[i][j], epsilon);
		//		}
		//	}
		//}

		//TEST_METHOD(InverseOrthogonalMatTest)
		//{
		//	Mat4f mat(
		//		Vec4f(0.0f, 1.0f, 0.0f, 0.0f),
		//		Vec4f(-1.0f, 0.0f, 0.0f, 0.0f),
		//		Vec4f(0.0f, 0.0f, 1.0f, 0.0f),
		//		Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
		//	);

		//	Mat4f expectedInverse(
		//		Vec4f(0.0f, -1.0f, 0.0f, 0.0f),
		//		Vec4f(1.0f, 0.0f, 0.0f, 0.0f),
		//		Vec4f(0.0f, 0.0f, 1.0f, 0.0f),
		//		Vec4f(0.0f, 0.0f, 0.0f, 1.0f)
		//	);

		//	Mat4f inverse;
		//	bool success = mat.Inverse(inverse);
		//	Assert::IsTrue(success);
		//	for (int i = 0; i < 4; i++)
		//	{
		//		for (int j = 0; j < 4; j++)
		//		{
		//			Assert::AreEqual(expectedInverse.m[i][j], inverse.m[i][j], epsilon);
		//		}
		//	}
		//}
	};
}