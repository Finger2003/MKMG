#pragma once

#include "../MathLib/Vec3f.h"
#include "../MathLib/Vec4f.h"
#include "../MathLib/Mat4f.h"

class SceneObject;
class BezierSurface;
class BSplineSurface;
struct float3;

namespace GeometryMath
{
	using MathLib::Vec3f;
	using MathLib::Vec4f;
	using MathLib::Mat4f;

	struct IntersectionResult
	{
		std::vector<Vec3f> points;
		std::vector<Vec4f> params;
		bool isClosedLoop;
		int forwardHitDim;
		int backwardHitDim;
	};

	struct SurfaceEvalResult
	{
		Vec3f p;
		Vec3f du;
		Vec3f dv;
	};

	struct CurveEvalResult
	{
		Vec3f p;
		Vec3f d;
	};

	struct AABB
	{
		Vec3f min_pt = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
		Vec3f max_pt = { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() };

		void Expand(const MathLib::Vec3f& p)
		{
			min_pt.x = std::min(min_pt.x, p.x); min_pt.y = std::min(min_pt.y, p.y); min_pt.z = std::min(min_pt.z, p.z);
			max_pt.x = std::max(max_pt.x, p.x); max_pt.y = std::max(max_pt.y, p.y); max_pt.z = std::max(max_pt.z, p.z);
		}

		bool Overlaps(const AABB& other, float tolerance = 1e-4f) const
		{
			if (max_pt.x < other.min_pt.x - tolerance || min_pt.x > other.max_pt.x + tolerance) return false;
			if (max_pt.y < other.min_pt.y - tolerance || min_pt.y > other.max_pt.y + tolerance) return false;
			if (max_pt.z < other.min_pt.z - tolerance || min_pt.z > other.max_pt.z + tolerance) return false;
			return true;
		}
	};

	struct SurfacePatch
	{
		float u_min, u_max, v_min, v_max;
		AABB aabb;
	};

#pragma region UI/Screen Math
	Vec3f ScreenToObjectArcballVector(int mouseX, int mouseY, float objScreenX, float objScreenY, float radius);
	std::pair<float, float> CalculateCoordsFromPixel(float x, float y, float width, float height);
#pragma endregion

#pragma region Parametric Evaluations
	CurveEvalResult EvaluateCubicDeCasteljau(float t, Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3);
	SurfaceEvalResult EvaluateBezierSurface(BezierSurface* surf, float u, float v);
	CurveEvalResult EvaluateCubicDeBoor(float t, Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3);
	SurfaceEvalResult EvaluateBSplineSurface(BSplineSurface* surf, float u, float v);
	SurfaceEvalResult EvaluateParametric(SceneObject* obj, float u, float v);
#pragma endregion

#pragma region Intersection Solvers
	std::optional<IntersectionResult> CalculateIntersection(
		SceneObject* obj1,
		SceneObject* obj2,
		bool isSelfIntersection,
		const MathLib::Vec4f& maxDomains, // x: maxU1, y: maxV1, z: maxU2, w: maxV2
		float step,
		std::optional<float3> cursorHint);

	std::optional<Vec4f> SolveLinearSystem(Mat4f J, Vec4f F);
	std::optional<Vec4f> FindIntersectionNextPoint(SceneObject* obj1, SceneObject* obj2, Vec4f startParams, Vec3f p0, Vec3f t, float d, int depth = 0);
	std::optional<Vec4f> FindExactEdgePoint(SceneObject* obj1, SceneObject* obj2, Vec4f startGuess, int boundaryDim, float boundaryValue);
#pragma endregion

#pragma region Patch Intersection
	AABB ComputePatchAABB(SceneObject* obj, float u_min, float u_max, float v_min, float v_max);
	void IntersectPatches(SceneObject* obj1, SurfacePatch p1, SceneObject* obj2, SurfacePatch p2, std::vector<Vec4f>& candidateParams, bool isSelfIntersection, int depth = 0);
#pragma endregion




#pragma region Implementation
	inline std::pair<float, float> CalculateCoordsFromPixel(float x, float y, float width, float height)
	{
		float normX = (x / width) * 2.0f - 1.0f;
		float normY = 1.0f - (y / height) * 2.0f; // Invert Y coordinate
		return { normX, normY };
	}

	inline CurveEvalResult EvaluateCubicDeCasteljau(float t, Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3)
	{
		float u = 1.0f - t;

		// Level 1
		Vec3f p01 = p0 * u + p1 * t;
		Vec3f p12 = p1 * u + p2 * t;
		Vec3f p23 = p2 * u + p3 * t;

		// Level 2
		Vec3f p012 = p01 * u + p12 * t;
		Vec3f p123 = p12 * u + p23 * t;

		// Level 3
		Vec3f p0123 = p012 * u + p123 * t;

		// Derivative calculation
		Vec3f derivative = (p123 - p012) * 3.0f;

		return { p0123, derivative };
	}

	inline CurveEvalResult EvaluateCubicDeBoor(float t, Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3)
	{
		// Level 1
		Vec3f p1_1 = p0 * ((1.0f - t) / 3.0f) + p1 * ((t + 2.0f) / 3.0f);
		Vec3f p2_1 = p1 * ((2.0f - t) / 3.0f) + p2 * ((t + 1.0f) / 3.0f);
		Vec3f p3_1 = p2 * ((3.0f - t) / 3.0f) + p3 * (t / 3.0f);

		// Level 2
		Vec3f p2_2 = p1_1 * ((1.0f - t) / 2.0f) + p2_1 * ((t + 1.0f) / 2.0f);
		Vec3f p3_2 = p2_1 * ((2.0f - t) / 2.0f) + p3_1 * (t / 2.0f);

		// Level 3
		Vec3f p3_3 = p2_2 * (1.0f - t) + p3_2 * t;

		// Derivative calculation
		Vec3f derivative = (p3_2 - p2_2) * 3.0f;

		return { p3_3, derivative };
	}
#pragma endregion
}
