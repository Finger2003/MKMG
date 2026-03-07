#include "pch.h"
#include "CppUnitTest.h"
#include "../MathLib/MathLib.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace MathLib;

namespace MathLibTest
{
	constexpr double epsilon = 1e-12;
	TEST_CLASS(MathLibTest)
	{
	public:
		TEST_METHOD(QuadraticTest)
		{
			double a = 1.0, b = -3.0, c = 2.0;
			double expected_r1 = 2.0, expected_r2 = 1.0;

			auto roots = SolveQuadratic(a, b, c);
			Assert::IsTrue(roots.has_value(), L"Expected real roots, but got none.");
			Assert::AreEqual(expected_r1, roots->r1, epsilon);
			Assert::AreEqual(expected_r2, roots->r2, epsilon);
		}

		TEST_METHOD(QuadraticNoRealRootsTest)
		{
			double a = 1.0, b = 0.0, c = 1.0; // Discriminant < 0
			auto roots = SolveQuadratic(a, b, c);
			Assert::IsFalse(roots.has_value(), L"Expected no real roots, but got some.");
		}

		TEST_METHOD(QuadraticLinearTest)
		{
			double a = 0.0, b = 2.0, c = -4.0; // Linear equation: 2x - 4 = 0
			double expected_root = 2.0;
			auto roots = SolveQuadratic(a, b, c);
			Assert::IsTrue(roots.has_value(), L"Expected a root for linear equation, but got none.");
			Assert::AreEqual(expected_root, roots->r1, epsilon);
			Assert::AreEqual(expected_root, roots->r2, epsilon);
		}

		TEST_METHOD(QuadraticDoubleRootTest)
		{
			double a = 1.0, b = -2.0, c = 1.0; // Discriminant = 0
			double expected_root = 1.0;
			auto roots = SolveQuadratic(a, b, c);
			Assert::IsTrue(roots.has_value(), L"Expected real roots, but got none.");
			Assert::AreEqual(expected_root, roots->r1, epsilon);
			Assert::AreEqual(expected_root, roots->r2, epsilon);
		}

		TEST_METHOD(QuadraticZerosTest)
		{
			double a = 1.0, b = 0.0, c = 0.0; // Roots at zero
			double expected_root = 0.0;
			auto roots = SolveQuadratic(a, b, c);
			Assert::IsTrue(roots.has_value(), L"Expected real roots, but got none.");
			Assert::AreEqual(expected_root, roots->r1, epsilon);
			Assert::AreEqual(expected_root, roots->r2, epsilon);
		}
	};
}