#include "pch.h"
#include "GeometryMath.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
#include "Torus.h"

using namespace MathLib;
using namespace GeometryMath;

MathLib::Vec3f GeometryMath::ScreenToObjectArcballVector(int mouseX, int mouseY, float objScreenX, float objScreenY, float radius)
{
	float dx = (mouseX - objScreenX) / radius;
	float dy = (objScreenY - mouseY) / radius; // Invert Y because screen Y goes down

	MathLib::Vec3f p(dx, dy, 0.0f);
	float length_sqr = p.length_sqr();

	if (length_sqr <= 0.5f)
		p.z = std::sqrt(1.0f - length_sqr);
	else
	{
		p.z = 0.5f / std::sqrt(length_sqr);
		p = p.normalize();
	}

	return p;
}

#pragma region Parametric Evaluations
SurfaceEvalResult GeometryMath::EvaluateBezierSurface(BezierSurface* surf, float u, float v)
{
	int segU = surf->GetSegmentsU();
	int segV = surf->GetSegmentsV();

	int pu = std::clamp(static_cast<int>(u), 0, segU - 1);
	int pv = std::clamp(static_cast<int>(v), 0, segV - 1);

	float lu = u - pu;
	float lv = v - pv;


	Vec3f P[4][4];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int idx = (pv * 3 + j) * surf->m_gridPointsU + (pu * 3 + i);
			if (auto pt = surf->m_controlPoints[idx].lock())
				P[i][j] = pt->m_position.ToVec3f();
			else
				P[i][j] = MathLib::Vec3f(0.0f, 0.0f, 0.0f);
		}
	}

	Vec3f Q[4];
	Vec3f dQu[4];
	for (int j = 0; j < 4; j++)
	{
		auto [pt, deriv] = EvaluateCubicDeCasteljau(lu, P[0][j], P[1][j], P[2][j], P[3][j]);
		Q[j] = pt;
		dQu[j] = deriv;
	}

	auto resV = EvaluateCubicDeCasteljau(lv, Q[0], Q[1], Q[2], Q[3]);
	auto resDu = EvaluateCubicDeCasteljau(lv, dQu[0], dQu[1], dQu[2], dQu[3]);

	return { resV.p, resDu.p, resV.d };
}

SurfaceEvalResult GeometryMath::EvaluateBSplineSurface(BSplineSurface* surf, float u, float v)
{
	int segU = surf->GetSegmentsU();
	int segV = surf->GetSegmentsV();

	int pu = std::clamp(static_cast<int>(u), 0, segU - 1);
	int pv = std::clamp(static_cast<int>(v), 0, segV - 1);

	float lu = u - pu;
	float lv = v - pv;

	Vec3f P[4][4];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			int idx = (pv + j) * surf->m_gridPointsU + (pu + i);
			if (auto pt = surf->m_controlPoints[idx].lock())
				P[i][j] = pt->m_position.ToVec3f();
			else
				P[i][j] = Vec3f(0.0f);
		}
	}

	Vec3f Q[4];
	Vec3f dQu[4];
	for (int j = 0; j < 4; j++)
	{
		// Evaluate De Boor in the U direction
		auto [pt, deriv] = EvaluateCubicDeBoor(lu, P[0][j], P[1][j], P[2][j], P[3][j]);
		Q[j] = pt;
		dQu[j] = deriv;
	}

	auto resV = EvaluateCubicDeBoor(lv, Q[0], Q[1], Q[2], Q[3]);
	auto resDu = EvaluateCubicDeBoor(lv, dQu[0], dQu[1], dQu[2], dQu[3]);

	return { resV.p, resDu.p, resV.d };
}

SurfaceEvalResult GeometryMath::EvaluateParametric(SceneObject* obj, float u, float v)
{
	if (obj->type == ObjectType::BezierSurface)
		return EvaluateBezierSurface(static_cast<BezierSurface*>(obj), u, v);
	else if (obj->type == ObjectType::BSplineSurface)
		return EvaluateBSplineSurface(static_cast<BSplineSurface*>(obj), u, v);
	else if (obj->type == ObjectType::Torus)
	{
		Torus* torus = static_cast<Torus*>(obj);
		constexpr float dBetaDu = 2.0f * std::numbers::pi_v<float>;
		constexpr float dAlphaDv = 2.0f * std::numbers::pi_v<float>;

		float beta = u * dBetaDu;
		float alpha = v * dAlphaDv;
		float R = torus->GetMajorRadius();
		float r = torus->GetMinorRadius();

		float cosAlpha = std::cos(alpha);
		float sinAlpha = std::sin(alpha);
		float cosBeta = std::cos(beta);
		float sinBeta = std::sin(beta);

		Vec4f pLocal(
			(R + r * cosAlpha) * cosBeta,
			r * sinAlpha,
			-(R + r * cosAlpha) * sinBeta,
			1.0f
		);

		Vec4f duLocal(
			-(R + r * cosAlpha) * sinBeta * dBetaDu,
			0.0f,
			-(R + r * cosAlpha) * cosBeta * dBetaDu,
			0.0f
		);

		Vec4f dvLocal(
			-r * sinAlpha * cosBeta * dAlphaDv,
			r * cosAlpha * dAlphaDv,
			r * sinAlpha * sinBeta * dAlphaDv,
			0.0f
		);

		Mat4f model = torus->m_modelMatrix;

		Vec4f pWorld = model * pLocal;
		Vec4f duWorld = model * duLocal;
		Vec4f dvWorld = model * dvLocal;

		return {
			Vec3f::FromVec4f(pWorld),
			Vec3f::FromVec4f(duWorld),
			Vec3f::FromVec4f(dvWorld)
		};
	}
	else
		return { Vec3f(0.0f), Vec3f(0.0f), Vec3f(0.0f) };
}
#pragma endregion

#pragma region Intersection Solvers
std::optional<MathLib::Vec4f> GeometryMath::SolveLinearSystem(MathLib::Mat4f J, MathLib::Vec4f F)
{
	for (int i = 0; i < 4; i++)
	{
		// Partial pivoting
		int pivot = i;
		for (int j = i + 1; j < 4; j++)
		{
			if (std::abs(J.m[j][i]) > std::abs(J.m[pivot][i]))
				pivot = j;
		}
		if (std::abs(J.m[pivot][i]) < 1e-8f)
			return std::nullopt; // singular matrix

		// Swap the current row with the pivot row
		std::swap(J.rows[i], J.rows[pivot]);
		std::swap(F.f[i], F.f[pivot]);

		// Scale the pivot row so the diagonal becomes 1
		float diag = J.m[i][i];
		float invDiag = 1.0f / diag;
		J.rows[i] *= invDiag;
		F.f[i] *= invDiag;

		// Eliminate the current variable from other rows
		for (int j = 0; j < 4; j++)
		{
			if (i != j)
			{
				float factor = J.m[j][i];
				J.rows[j] = J.rows[j] - (J.rows[i] * factor);
				F.f[j] -= factor * F.f[i];
			}
		}
	}

	return F;
}

std::optional<MathLib::Vec4f> GeometryMath::FindIntersectionNextPoint(SceneObject* obj1, SceneObject* obj2, MathLib::Vec4f startParams, MathLib::Vec3f p0, MathLib::Vec3f t, float d, int depth)
{
	constexpr int maxDepth = 3;
	if (depth > maxDepth)
		return std::nullopt;

	Vec4f currentParams = startParams;
	bool converged = false;
	constexpr int maxIterations = 50;
	constexpr float F_dist_tolerance = 1e-8f;
	constexpr float planeTolerance = 1e-4f;

	for (int iter = 0; iter < maxIterations; iter++)
	{
		auto [p1, du1, dv1] = EvaluateParametric(obj1, currentParams.x, currentParams.y);
		auto [p2, du2, dv2] = EvaluateParametric(obj2, currentParams.z, currentParams.w);

		Vec3f F_dist = p1 - p2;
		float planeDist = Vec3f::dot(p1 - p0, t) - d;

		if (F_dist.length_sqr() < F_dist_tolerance && std::abs(planeDist) < planeTolerance)
		{
			converged = true;
			break;
		}

		Mat4f J(
			Vec4f(du1.x, dv1.x, -du2.x, -dv2.x),
			Vec4f(du1.y, dv1.y, -du2.y, -dv2.y),
			Vec4f(du1.z, dv1.z, -du2.z, -dv2.z),
			Vec4f(Vec3f::dot(du1, t), Vec3f::dot(dv1, t), 0.0f, 0.0f)
		);

		Vec4f F(-F_dist.x, -F_dist.y, -F_dist.z, -planeDist);

		auto deltaOpt = SolveLinearSystem(J, F);
		if (!deltaOpt)
			break;

		currentParams += *deltaOpt;
	}

	if (converged)
		return currentParams;
	else
	{
		auto halfStep1 = FindIntersectionNextPoint(obj1, obj2, startParams, p0, t, d / 2.0f, depth + 1);
		if (!halfStep1)
			return std::nullopt;

		auto [p_mid, _1, _2] = EvaluateParametric(obj1, halfStep1->x, halfStep1->y);

		auto halfStep2 = FindIntersectionNextPoint(obj1, obj2, *halfStep1, p_mid, t, d / 2.0f, depth + 1);
		return halfStep2;
	}
}

std::optional<MathLib::Vec4f> GeometryMath::FindExactEdgePoint(SceneObject* obj1, SceneObject* obj2, MathLib::Vec4f startGuess, int boundaryDim, float boundaryValue)
{
	Vec4f currentParams = startGuess;
	currentParams.f[boundaryDim] = boundaryValue;
	constexpr int maxIterations = 100;
	constexpr float F_dist_tolerance = 1e-8f;

	for (int iter = 0; iter < maxIterations; iter++)
	{
		auto [p1, du1, dv1] = EvaluateParametric(obj1, currentParams.x, currentParams.y);
		auto [p2, du2, dv2] = EvaluateParametric(obj2, currentParams.z, currentParams.w);
		Vec3f F_dist = p1 - p2;

		if (F_dist.length_sqr() < F_dist_tolerance)
			return currentParams;

		Mat4f J(
			Vec4f(du1.x, dv1.x, -du2.x, -dv2.x),
			Vec4f(du1.y, dv1.y, -du2.y, -dv2.y),
			Vec4f(du1.z, dv1.z, -du2.z, -dv2.z),
			Vec4f(0.0f, 0.0f, 0.0f, 0.0f)
		);

		J.m[3][boundaryDim] = 1.0f; // Enforce boundary condition

		Vec4f F(-F_dist.x, -F_dist.y, -F_dist.z, -(currentParams.f[boundaryDim] - boundaryValue));

		auto deltaOpt = SolveLinearSystem(J, F);
		if (!deltaOpt)
			return std::nullopt;

		currentParams += *deltaOpt;
		currentParams.f[boundaryDim] = boundaryValue;
	}

	return std::nullopt;
}
#pragma endregion

#pragma region Patch Intersection
AABB GeometryMath::ComputePatchAABB(SceneObject* obj, float u_min, float u_max, float v_min, float v_max)
{
	AABB box;
	float max_du_len = 0.0f;
	float max_dv_len = 0.0f;

	for (int i = 0; i <= 2; i++)
	{
		float u = std::lerp(u_min, u_max, i / 2.0f);
		for (int j = 0; j <= 2; j++)
		{
			float v = std::lerp(v_min, v_max, j / 2.0f);

			auto eval = EvaluateParametric(obj, u, v);
			box.Expand(eval.p);

			if (obj->IsA(ObjectType::Surface))
			{
				max_du_len = std::max(max_du_len, eval.du.length());
				max_dv_len = std::max(max_dv_len, eval.dv.length());
			}
		}
	}

	float padding = 0.0f;
	if (obj->type == ObjectType::Torus)
	{
		auto torus = static_cast<Torus*>(obj);

		float stepU = (u_max - u_min) / 2.0f;
		float stepV = (v_max - v_min) / 2.0f;

		float thetaU = stepU * 2.0f * std::numbers::pi_v<float>;
		float thetaV = stepV * 2.0f * std::numbers::pi_v<float>;

		float maxRadiusU = torus->GetMajorRadius() + torus->GetMinorRadius();
		float sagittaU = maxRadiusU * (1.0f - std::cos(thetaU / 2.0f));

		float maxRadiusV = torus->GetMinorRadius();
		float sagittaV = maxRadiusV * (1.0f - std::cos(thetaV / 2.0f));

		padding = sagittaU + sagittaV;
	}
	else if (obj->IsA(ObjectType::Surface))
	{
		float stepU = (u_max - u_min) / 2.0f;
		float stepV = (v_max - v_min) / 2.0f;

		padding = (max_du_len * stepU + max_dv_len * stepV) * 1.2f;
	}

	box.min_pt = box.min_pt - MathLib::Vec3f(padding + 1e-5f, padding + 1e-5f, padding + 1e-5f);
	box.max_pt = box.max_pt + MathLib::Vec3f(padding + 1e-5f, padding + 1e-5f, padding + 1e-5f);

	return box;
}
void GeometryMath::IntersectPatches(SceneObject* obj1, SurfacePatch p1, SceneObject* obj2, SurfacePatch p2, std::vector<MathLib::Vec4f>& candidateParams, bool isSelfIntersection, int depth)
{
	// Bounding boxes don't overlap, cut off this branch
	if (!p1.aabb.Overlaps(p2.aabb))
		return;

	// Max depth reached, add the center of the patches as a candidate
	constexpr int maxDepth = 6;
	if (depth >= maxDepth)
	{
		if (isSelfIntersection)
		{
			constexpr float eps = 1e-4f;
			bool separatedU = (p1.u_max < p2.u_min - eps) || (p2.u_max < p1.u_min - eps);
			bool separatedV = (p1.v_max < p2.v_min - eps) || (p2.v_max < p1.v_min - eps);
			if (!separatedU && !separatedV)
				return;
		}

		float u1 = (p1.u_min + p1.u_max) * 0.5f;
		float v1 = (p1.v_min + p1.v_max) * 0.5f;
		float u2 = (p2.u_min + p2.u_max) * 0.5f;
		float v2 = (p2.v_min + p2.v_max) * 0.5f;
		candidateParams.push_back({ u1, v1, u2, v2 });
		return;
	}

	// Subdivide the larger patch into 4 quadrants
	float diag1 = (p1.aabb.max_pt - p1.aabb.min_pt).length_sqr();
	float diag2 = (p2.aabb.max_pt - p2.aabb.min_pt).length_sqr();

	if (diag1 > diag2)
	{
		float midU = (p1.u_min + p1.u_max) * 0.5f;
		float midV = (p1.v_min + p1.v_max) * 0.5f;

		SurfacePatch sub[4] = {
			{ p1.u_min, midU, p1.v_min, midV, ComputePatchAABB(obj1, p1.u_min, midU, p1.v_min, midV) },
			{ midU, p1.u_max, p1.v_min, midV, ComputePatchAABB(obj1, midU, p1.u_max, p1.v_min, midV) },
			{ p1.u_min, midU, midV, p1.v_max, ComputePatchAABB(obj1, p1.u_min, midU, midV, p1.v_max) },
			{ midU, p1.u_max, midV, p1.v_max, ComputePatchAABB(obj1, midU, p1.u_max, midV, p1.v_max) }
		};

		for (int i = 0; i < 4; i++)
			IntersectPatches(obj1, sub[i], obj2, p2, candidateParams, isSelfIntersection, depth + 1);
	}
	else
	{
		float midU = (p2.u_min + p2.u_max) * 0.5f;
		float midV = (p2.v_min + p2.v_max) * 0.5f;

		SurfacePatch sub[4] = {
			{ p2.u_min, midU, p2.v_min, midV, ComputePatchAABB(obj2, p2.u_min, midU, p2.v_min, midV) },
			{ midU, p2.u_max, p2.v_min, midV, ComputePatchAABB(obj2, midU, p2.u_max, p2.v_min, midV) },
			{ p2.u_min, midU, midV, p2.v_max, ComputePatchAABB(obj2, p2.u_min, midU, midV, p2.v_max) },
			{ midU, p2.u_max, midV, p2.v_max, ComputePatchAABB(obj2, midU, p2.u_max, midV, p2.v_max) }
		};

		for (int i = 0; i < 4; i++)
			IntersectPatches(obj1, p1, obj2, sub[i], candidateParams, isSelfIntersection, depth + 1);
	}
}
#pragma endregion
