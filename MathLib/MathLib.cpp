#include "pch.h"
#include "MathLib.h"

using namespace MathLib;

std::optional<QuadraticRoots> MathLib::SolveQuadratic(double a, double b, double c, double epsilon)
{
	if (std::abs(a) < epsilon)
	{
		if (std::abs(b) < epsilon)
			return std::nullopt;

		double root = -c / b;
		return QuadraticRoots{ root, root }; // Linear solution.
	}

	double discriminant = std::fma(b, b, -4.0 * a * c);
	if (discriminant < -epsilon)
		return std::nullopt; // No real roots.

	double sqrt_disc = std::sqrt(std::max(discriminant, 0.0));
	double sign_b = (b >= 0.0) ? 1.0 : -1.0;
	double q = -0.5 * (b + sign_b * sqrt_disc);

	if (std::abs(q) < epsilon)
		return QuadraticRoots{ 0.0, 0.0 };

	double x1 = q / a;
	double x2 = c / q;

	return QuadraticRoots{ x1, x2 };
}
