#pragma once
#include "../MathLib/Vec3f.h"
namespace SplineMath
{
    inline MathLib::Vec3f EvaluateBSpline1D(const MathLib::Vec3f& p0, const MathLib::Vec3f& p1, const MathLib::Vec3f& p2, const MathLib::Vec3f& p3, int index)
    {
        switch (index)
        {
        case 0: return (p0 + 4.0f * p1 + p2) / 6.0f;
        case 1: return (4.0f * p1 + 2.0f * p2) / 6.0f;
        case 2: return (2.0f * p1 + 4.0f * p2) / 6.0f;
        case 3: return (p1 + 4.0f * p2 + p3) / 6.0f;
        default: return { 0.0f, 0.0f, 0.0f };
        }
    }
}