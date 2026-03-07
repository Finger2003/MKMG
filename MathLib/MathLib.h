#pragma once
#include <optional>

namespace MathLib
{
	struct QuadraticRoots
	{
		double r1, r2;
		double max_root() const { return std::max(r1, r2); }
	};

	std::optional<QuadraticRoots> SolveQuadratic(double a, double b, double c, double epsilon = 1e-12);
}