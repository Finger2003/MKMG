#include "pch.h"
#include "GeometryMath.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
#include "Torus.h"

using namespace MathLib;
using namespace GeometryMath;

namespace
{
	struct Candidate
	{
		MathLib::Vec4f params;
		float dist;
	};

	float GetMaxU(SceneObject* obj)
	{
		if (auto surf = obj->As<Surface>())
			return static_cast<float>(surf->GetSegmentsU());
		return 1.0f;
	}

	float GetMaxV(SceneObject* obj)
	{
		if (auto surf = obj->As<Surface>())
			return static_cast<float>(surf->GetSegmentsV());
		return 1.0f;
	}

	bool IsSeamConnected(SceneObject* obj, bool inU, float otherParam)
	{
		if (obj->type == ObjectType::Torus)
			return true;
		if (!obj->IsA(ObjectType::Surface))
			return false;
		if (std::isnan(otherParam))
			return false;

		float maxU = GetMaxU(obj);
		float maxV = GetMaxV(obj);
		float maxOther = inU ? maxV : maxU;
		float oc = std::clamp(otherParam, 0.0f, maxOther);

		auto boundaryDistSqr = [&](float coord) {
			Vec3f a = inU ? EvaluateParametric(obj, 0.0f, coord).p : EvaluateParametric(obj, coord, 0.0f).p;
			Vec3f b = inU ? EvaluateParametric(obj, maxU, coord).p : EvaluateParametric(obj, coord, maxV).p;
			return (a - b).length_sqr();
		};

		if (boundaryDistSqr(oc) >= 1e-6f)
			return false;

		if (oc <= 1e-4f || oc >= maxOther - 1e-4f)
		{
			float oi = (oc <= 1e-4f) ? std::min(maxOther, 0.01f) : std::max(0.0f, maxOther - 0.01f);
			if (boundaryDistSqr(oi) >= 1e-6f)
				return false;
		}

		return true;
	}

	bool IsSeamDim(SceneObject* obj1, SceneObject* obj2, const Vec4f& params, int dim)
	{
		SceneObject* obj = (dim < 2) ? obj1 : obj2;
		bool inU = (dim == 0 || dim == 2);
		float otherParam = params.f[inU ? (dim + 1) : (dim - 1)];
		return IsSeamConnected(obj, inU, otherParam);
	}

	void WrapSeamConnectedParams(SceneObject* obj, float maxU, float maxV, float& u, float& v)
	{
		if ((u < 0.0f || u > maxU) && IsSeamConnected(obj, true, v))
		{
			u = std::fmod(u, maxU);
			if (u < 0.0f)
				u += maxU;
		}
		if ((v < 0.0f || v > maxV) && IsSeamConnected(obj, false, u))
		{
			v = std::fmod(v, maxV);
			if (v < 0.0f)
				v += maxV;
		}
	}

	std::vector<Candidate> FindIntersectionCandidates(SceneObject* obj1, SceneObject* obj2, bool isSelfIntersection, const MathLib::Vec4f& maxDomains, std::optional<float3> cursorHint)
	{
		float maxU1 = maxDomains.x;
		float maxV1 = maxDomains.y;
		float maxU2 = maxDomains.z;
		float maxV2 = maxDomains.w;

		std::vector<Candidate> sortedCandidates;

		if (cursorHint)
		{
			constexpr int cursorSamples = 50;
			Vec3f cursorPos = cursorHint->ToVec3f();

			float minVal1 = std::numeric_limits<float>::max();
			float minVal2 = std::numeric_limits<float>::max();
			Vec4f currentParams{ 0.0f, 0.0f, 0.0f, 0.0f };

			if (isSelfIntersection)
			{
				for (int i = 0; i <= cursorSamples; i++)
				{
					float u = (i / static_cast<float>(cursorSamples)) * maxU1;
					for (int j = 0; j <= cursorSamples; j++)
					{
						float v = (j / static_cast<float>(cursorSamples)) * maxV1;
						float dist = (EvaluateParametric(obj1, u, v).p - cursorPos).length_sqr();

						if (dist < minVal1)
						{
							float pDist = (u - currentParams.x) * (u - currentParams.x) + (v - currentParams.y) * (v - currentParams.y);
							float domainDiagonalSqr = (maxU1 * maxU1) + (maxV1 * maxV1);
							float dynamicThreshold = domainDiagonalSqr * 0.05f;
							if (pDist > dynamicThreshold)
							{
								minVal2 = minVal1;
								currentParams.z = currentParams.x;
								currentParams.w = currentParams.y;
							}
							minVal1 = dist;
							currentParams.x = u;
							currentParams.y = v;
						}
						else if (dist < minVal2)
						{
							float pDist = (u - currentParams.x) * (u - currentParams.x) + (v - currentParams.y) * (v - currentParams.y);
							float domainDiagonalSqr = (maxU1 * maxU1) + (maxV1 * maxV1);
							float dynamicThreshold = domainDiagonalSqr * 0.05f;
							if (pDist > dynamicThreshold)
							{
								minVal2 = dist;
								currentParams.z = u;
								currentParams.w = v;
							}
						}
					}
				}
			}
			else
			{
				for (int i = 0; i <= cursorSamples; i++)
				{
					float u1 = (i / static_cast<float>(cursorSamples)) * maxU1;
					float u2 = (i / static_cast<float>(cursorSamples)) * maxU2;

					for (int j = 0; j <= cursorSamples; j++)
					{
						float v1 = (j / static_cast<float>(cursorSamples)) * maxV1;
						float v2 = (j / static_cast<float>(cursorSamples)) * maxV2;

						float dist1 = (EvaluateParametric(obj1, u1, v1).p - cursorPos).length_sqr();
						if (dist1 < minVal1)
						{
							minVal1 = dist1;
							currentParams.x = u1;
							currentParams.y = v1;
						}

						float dist2 = (EvaluateParametric(obj2, u2, v2).p - cursorPos).length_sqr();
						if (dist2 < minVal2)
						{
							minVal2 = dist2;
							currentParams.z = u2;
							currentParams.w = v2;
						}
					}
				}
			}

			auto p1 = EvaluateParametric(obj1, currentParams.x, currentParams.y).p;
			auto p2 = EvaluateParametric(obj2, currentParams.z, currentParams.w).p;
			sortedCandidates.push_back({ currentParams, (p1 - p2).length_sqr() });

			std::vector<MathLib::Vec4f> patchParams;
			SurfacePatch root1 = { 0.0f, maxU1, 0.0f, maxV1, ComputePatchAABB(obj1, 0.0f, maxU1, 0.0f, maxV1) };
			SurfacePatch root2 = { 0.0f, maxU2, 0.0f, maxV2, ComputePatchAABB(obj2, 0.0f, maxU2, 0.0f, maxV2) };
			IntersectPatches(obj1, root1, obj2, root2, patchParams, isSelfIntersection);

			std::vector<Candidate> patchCandidates;
			patchCandidates.reserve(patchParams.size());
			for (const auto& params : patchParams)
			{
				auto cp1 = EvaluateParametric(obj1, params.x, params.y).p;
				auto cp2 = EvaluateParametric(obj2, params.z, params.w).p;
				Vec3f mid = (cp1 + cp2) * 0.5f;
				float score = (mid - cursorPos).length_sqr() + (cp1 - cp2).length_sqr();
				patchCandidates.push_back({ params, score });
			}

			std::sort(patchCandidates.begin(), patchCandidates.end(), [](const Candidate& a, const Candidate& b) {
				return a.dist < b.dist;
				});

			sortedCandidates.insert(sortedCandidates.end(), patchCandidates.begin(), patchCandidates.end());
		}
		else
		{
			std::vector<MathLib::Vec4f> candidateParams;
			SurfacePatch root1 = { 0.0f, maxU1, 0.0f, maxV1, ComputePatchAABB(obj1, 0.0f, maxU1, 0.0f, maxV1) };
			SurfacePatch root2 = { 0.0f, maxU2, 0.0f, maxV2, ComputePatchAABB(obj2, 0.0f, maxU2, 0.0f, maxV2) };

			IntersectPatches(obj1, root1, obj2, root2, candidateParams, isSelfIntersection);

			sortedCandidates.reserve(candidateParams.size());
			for (const auto& params : candidateParams)
			{
				auto p1 = EvaluateParametric(obj1, params.x, params.y).p;
				auto p2 = EvaluateParametric(obj2, params.z, params.w).p;
				sortedCandidates.push_back({ params, (p1 - p2).length_sqr() });
			}

			std::sort(sortedCandidates.begin(), sortedCandidates.end(), [](const Candidate& a, const Candidate& b) {
				return a.dist < b.dist;
				});
		}

		return sortedCandidates;
	}

	std::optional<Vec4f> RefineIntersectionPoint(SceneObject* obj1, SceneObject* obj2, Vec4f startParams, const Vec4f& maxDomains, bool isSelfIntersection)
	{
		float maxU1 = maxDomains.x;
		float maxV1 = maxDomains.y;
		float maxU2 = maxDomains.z;
		float maxV2 = maxDomains.w;

		auto applyBounds = [&](Vec4f& p) {
			for (int d = 0; d < 4; d++)
			{
				float maxVal = maxDomains.f[d];
				if (p.f[d] < 0.0f || p.f[d] > maxVal)
				{
					if (IsSeamDim(obj1, obj2, p, d))
					{
						p.f[d] = std::fmod(p.f[d], maxVal);
						if (p.f[d] < 0.0f)
							p.f[d] += maxVal;
					}
					else
					{
						p.f[d] = std::clamp(p.f[d], 0.0f, maxVal);
					}
				}
			}
			};

		Vec4f currentParams = startParams;
		float alpha = 0.5f;
		constexpr float alphaIncrease = 1.2f;
		constexpr float alphaReduction = 0.5f;
		constexpr float tolerance = 1e-8f;
		constexpr int maxIterations = 100;

		SurfaceEvalResult eval1 = EvaluateParametric(obj1, currentParams.x, currentParams.y);
		SurfaceEvalResult eval2 = EvaluateParametric(obj2, currentParams.z, currentParams.w);

		for (int iter = 0; iter < maxIterations; iter++)
		{
			Vec3f D = eval1.p - eval2.p;
			float distSqr = D.length_sqr();
			if (distSqr < tolerance)
				break;

			bool accepted = false;

			if (!isSelfIntersection)
			{
				Vec3f np = Vec3f::cross(eval1.du, eval1.dv).normalize();
				Vec3f nq = Vec3f::cross(eval2.du, eval2.dv).normalize();
				Vec3f t = Vec3f::cross(np, nq);

				if (t.length_sqr() > 1e-8f)
				{
					Mat4f J(
						Vec4f(eval1.du.x, eval1.dv.x, -eval2.du.x, -eval2.dv.x),
						Vec4f(eval1.du.y, eval1.dv.y, -eval2.du.y, -eval2.dv.y),
						Vec4f(eval1.du.z, eval1.dv.z, -eval2.du.z, -eval2.dv.z),
						Vec4f(Vec3f::dot(eval1.du, t), Vec3f::dot(eval1.dv, t), 0.0f, 0.0f)
					);
					Vec4f F(-D.x, -D.y, -D.z, 0.0f);

					if (auto deltaOpt = SolveLinearSystem(J, F))
					{
						for (float scale : { 1.0f, 0.5f, 0.25f, 0.1f, 0.02f })
						{
							Vec4f candParams = currentParams + (*deltaOpt) * scale;
							applyBounds(candParams);

							auto candEval1 = EvaluateParametric(obj1, candParams.x, candParams.y);
							auto candEval2 = EvaluateParametric(obj2, candParams.z, candParams.w);

							if ((candEval1.p - candEval2.p).length_sqr() < distSqr)
							{
								currentParams = candParams;
								eval1 = candEval1;
								eval2 = candEval2;
								accepted = true;
								break;
							}
						}
					}
				}
			}

			if (!accepted)
			{
				float gu = Vec3f::dot(D, eval1.du) / std::max(eval1.du.length_sqr(), 1e-4f);
				float gv = Vec3f::dot(D, eval1.dv) / std::max(eval1.dv.length_sqr(), 1e-4f);
				float gs = Vec3f::dot(-D, eval2.du) / std::max(eval2.du.length_sqr(), 1e-4f);
				float gt = Vec3f::dot(-D, eval2.dv) / std::max(eval2.dv.length_sqr(), 1e-4f);

				Vec4f nextParams = currentParams - alpha * Vec4f(gu, gv, gs, gt);
				applyBounds(nextParams);

				auto nextEval1 = EvaluateParametric(obj1, nextParams.x, nextParams.y);
				auto nextEval2 = EvaluateParametric(obj2, nextParams.z, nextParams.w);

				if ((nextEval1.p - nextEval2.p).length_sqr() < distSqr)
				{
					currentParams = nextParams;
					eval1 = nextEval1;
					eval2 = nextEval2;
					alpha = std::min(alpha * alphaIncrease, 1.0f);
				}
				else
				{
					alpha *= alphaReduction;
				}

				if (alpha < 1e-6f)
					break;
			}
		}

		if ((eval1.p - eval2.p).length_sqr() <= 1e-5f)
		{
			if (isSelfIntersection)
			{
				float pDist = (currentParams.x - currentParams.z) * (currentParams.x - currentParams.z) +
					(currentParams.y - currentParams.w) * (currentParams.y - currentParams.w);
				if (pDist < 1e-4f)
					return std::nullopt; // converged to the same point on the surface
			}

			Vec3f np = Vec3f::cross(eval1.du, eval1.dv).normalize();
			Vec3f nq = Vec3f::cross(eval2.du, eval2.dv).normalize();
			float intersectionStrength = Vec3f::cross(np, nq).length_sqr();

			if (intersectionStrength >= 0.0025f)
				return currentParams;
		}

		return std::nullopt;
	}

	std::optional<IntersectionResult> TraceIntersectionPath(SceneObject* obj1, SceneObject* obj2, Vec4f startParams, float step, const Vec4f& maxDomains, bool isSelfIntersection)
	{
		float maxU1 = maxDomains.x;
		float maxV1 = maxDomains.y;
		float maxU2 = maxDomains.z;
		float maxV2 = maxDomains.w;

		auto isOutOfBounds = [&](const Vec4f& params) {
			return params.x < 0.0f || params.x > maxU1 || params.y < 0.0f || params.y > maxV1 ||
				params.z < 0.0f || params.z > maxU2 || params.w < 0.0f || params.w > maxV2;
			};

		auto startEval = EvaluateParametric(obj1, startParams.x, startParams.y);
		Vec3f startPos = startEval.p;

		std::vector<Vec3f> forwardPoints;
		std::vector<Vec3f> backwardPoints;
		std::vector<Vec4f> forwardParams;
		std::vector<Vec4f> backwardParams;
		bool isClosedLoop = false;
		bool forwardHitEdge = false;
		bool backwardHitEdge = false;
		int forwardHitDim = -1;
		int backwardHitDim = -1;
		Vec3f startTangent{ 0.0f, 0.0f, 0.0f };

		for (float direction : { 1.0f, -1.0f })
		{
			if (isClosedLoop)
				break;

			Vec4f marchingParams = startParams;
			Vec3f prevTangent{ 0.0f, 0.0f, 0.0f };
			Vec3f prevPos = startPos;
			bool leftStartRadius = false;
			int stepCount = 0;
			constexpr int maxMarchSteps = 10000;

			for (int iterGuard = 0; iterGuard < maxMarchSteps; iterGuard++)
			{
				auto [p1, du1, dv1] = EvaluateParametric(obj1, marchingParams.x, marchingParams.y);
				auto [p2, du2, dv2] = EvaluateParametric(obj2, marchingParams.z, marchingParams.w);

				Vec3f np = Vec3f::cross(du1, dv1).normalize();
				Vec3f nq = Vec3f::cross(du2, dv2).normalize();
				Vec3f t = Vec3f::cross(np, nq);

				float tLen = t.length();

				if (tLen < 0.05f)
				{
					if (stepCount == 0)
						break;
					t = prevTangent;
				}
				else
				{
					t = (t / tLen) * direction;
					if (stepCount == 0 && direction == 1.0f)
						startTangent = t;
				}
				prevTangent = t;

				auto nextParamsOpt = FindIntersectionNextPoint(obj1, obj2, marchingParams, p1, t, step);

				if (!nextParamsOpt)
					break;

				auto [v_p1, v_du1, v_dv1] = EvaluateParametric(obj1, nextParamsOpt->x, nextParamsOpt->y);
				auto [v_p2, v_du2, v_dv2] = EvaluateParametric(obj2, nextParamsOpt->z, nextParamsOpt->w);
				if ((v_p1 - v_p2).length_sqr() > 1e-5f)
					break;

				if (isSelfIntersection)
				{
					float pDist = (nextParamsOpt->x - nextParamsOpt->z) * (nextParamsOpt->x - nextParamsOpt->z) +
						(nextParamsOpt->y - nextParamsOpt->w) * (nextParamsOpt->y - nextParamsOpt->w);
					if (pDist < 1e-4f)
						break;
				}

				if (isOutOfBounds(*nextParamsOpt))
				{
					Vec4f P0 = marchingParams;
					Vec4f P1 = *nextParamsOpt;
					Vec4f Delta = P1 - P0;
					float T = 1.0f;
					int hitDim = -1;
					float hitTarget = 0.0f;

					auto checkBoundary = [&](int dim, float maxVal) {
						float p0 = P0.f[dim];
						float p1 = P1.f[dim];
						float d = Delta.f[dim];
						if (std::abs(d) > 1e-8f)
						{
							if (p1 > maxVal)
							{
								float t = (maxVal - p0) / d;
								if (t < T) { T = t; hitDim = dim; hitTarget = maxVal; }
							}
							else if (p1 < 0.0f)
							{
								float t = (0.0f - p0) / d;
								if (t < T) { T = t; hitDim = dim; hitTarget = 0.0f; }
							}
						}
						};

					checkBoundary(0, maxU1);
					checkBoundary(1, maxV1);
					checkBoundary(2, maxU2);
					checkBoundary(3, maxV2);

					if (hitDim != -1)
					{
						Vec4f guess = P0 + Delta * T;
						auto exactEdgeOpt = FindExactEdgePoint(obj1, obj2, guess, hitDim, hitTarget);

						Vec4f finalParams = guess;
						bool exactValid = false;

						if (exactEdgeOpt)
						{
							auto isStrictlyOutOfBounds = [&](const Vec4f& p) {
								constexpr float eps = 1e-4f;
								return p.x < -eps || p.x > maxU1 + eps ||
									p.y < -eps || p.y > maxV1 + eps ||
									p.z < -eps || p.z > maxU2 + eps ||
									p.w < -eps || p.w > maxV2 + eps;
								};
							if (!isStrictlyOutOfBounds(*exactEdgeOpt))
							{
								finalParams = *exactEdgeOpt;
								finalParams.x = std::clamp(finalParams.x, 0.0f, maxU1);
								finalParams.y = std::clamp(finalParams.y, 0.0f, maxV1);
								finalParams.z = std::clamp(finalParams.z, 0.0f, maxU2);
								finalParams.w = std::clamp(finalParams.w, 0.0f, maxV2);
								finalParams.f[hitDim] = hitTarget;
								exactValid = true;
							}
						}
						if (!exactValid)
						{
							auto [g_p1, g_du1, g_dv1] = EvaluateParametric(obj1, guess.x, guess.y);
							auto [g_p2, g_du2, g_dv2] = EvaluateParametric(obj2, guess.z, guess.w);
							if ((g_p1 - g_p2).length_sqr() > 1e-5f)
								break;
						}

						auto [edgePos, _u, _v] = EvaluateParametric(obj1, finalParams.x, finalParams.y);
						if ((edgePos - startPos).length_sqr() >= (step * step))
							leftStartRadius = true;

						Vec4f oppositeParams = finalParams;
						float oppositeTarget = 0.0f;

						if (hitTarget <= 0.0f)
						{
							if (hitDim == 0) oppositeTarget = maxU1;
							else if (hitDim == 1) oppositeTarget = maxV1;
							else if (hitDim == 2) oppositeTarget = maxU2;
							else if (hitDim == 3) oppositeTarget = maxV2;
						}
						oppositeParams.f[hitDim] = oppositeTarget;

						Vec3f seamEdgePos, seamOppositePos;
						if (hitDim == 0 || hitDim == 1)
						{
							seamEdgePos = EvaluateParametric(obj1, finalParams.x, finalParams.y).p;
							seamOppositePos = EvaluateParametric(obj1, oppositeParams.x, oppositeParams.y).p;
						}
						else
						{
							seamEdgePos = EvaluateParametric(obj2, finalParams.z, finalParams.w).p;
							seamOppositePos = EvaluateParametric(obj2, oppositeParams.z, oppositeParams.w).p;
						}

						// If the opposite edge physically touches this edge, it's a seam
						if ((seamOppositePos - seamEdgePos).length_sqr() < 1e-6f)
						{
							if (direction == 1.0f)
							{
								forwardPoints.push_back(edgePos);
								forwardParams.push_back(finalParams);
							}
							else
							{
								backwardPoints.push_back(edgePos);
								backwardParams.push_back(finalParams);
							}

							marchingParams = oppositeParams;
							prevPos = edgePos;
							stepCount++;
							continue;
						}

						if (direction == 1.0f)
						{
							forwardPoints.push_back(edgePos);
							forwardParams.push_back(finalParams);
							forwardHitEdge = true;
							forwardHitDim = hitDim;
						}
						else
						{
							backwardPoints.push_back(edgePos);
							backwardParams.push_back(finalParams);
							backwardHitEdge = true;
							backwardHitDim = hitDim;
						}
					}

					break; // hit the edge, stop marching in this direction
				}

				marchingParams = *nextParamsOpt;
				auto [newPos, _u, _v] = EvaluateParametric(obj1, marchingParams.x, marchingParams.y);
				float stepDistSqr = (newPos - prevPos).length_sqr();
				float distToStartSqr = (newPos - startPos).length_sqr();
				if (distToStartSqr >= (step * step))
					leftStartRadius = true;

				if (direction == 1.0f && stepCount > 3 && leftStartRadius && distToStartSqr <= std::max(step * step, stepDistSqr * 1.5f) && Vec3f::dot(t, startTangent) > 0.0f)
				{
					isClosedLoop = true;
					forwardPoints.push_back(startPos);
					forwardParams.push_back(marchingParams);
					break;
				}

				if (direction == 1.0f)
				{
					forwardPoints.push_back(newPos);
					forwardParams.push_back(marchingParams);
				}
				else
				{
					backwardPoints.push_back(newPos);
					backwardParams.push_back(marchingParams);
				}

				prevPos = newPos;
				stepCount++;
			}
		}

		if (!isClosedLoop && (!forwardHitEdge || !backwardHitEdge))
			return std::nullopt;

		IntersectionResult result;
		result.isClosedLoop = isClosedLoop;
		result.forwardHitDim = forwardHitDim;
		result.backwardHitDim = backwardHitDim;

		result.points.reserve(backwardPoints.size() + forwardPoints.size() + 1);
		result.points.insert(result.points.end(), backwardPoints.rbegin(), backwardPoints.rend());
		result.points.push_back(startPos);
		result.points.insert(result.points.end(), forwardPoints.begin(), forwardPoints.end());

		result.params.reserve(backwardParams.size() + forwardParams.size() + 1);
		result.params.insert(result.params.end(), backwardParams.rbegin(), backwardParams.rend());
		result.params.push_back(startParams);
		result.params.insert(result.params.end(), forwardParams.begin(), forwardParams.end());

		return result;
	}
}

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
	float maxU = static_cast<float>(segU);
	float maxV = static_cast<float>(segV);

	WrapSeamConnectedParams(surf, maxU, maxV, u, v);

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
	float maxU = static_cast<float>(segU);
	float maxV = static_cast<float>(segV);

	WrapSeamConnectedParams(surf, maxU, maxV, u, v);

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
std::optional<IntersectionResult> GeometryMath::CalculateIntersection(SceneObject* obj1, SceneObject* obj2, bool isSelfIntersection, const MathLib::Vec4f& maxDomains, float step, std::optional<float3> cursorHint)
{
	// =============================================================
	// STAGE 1: FIND STARTING PARAMETERS
	auto candidates = FindIntersectionCandidates(obj1, obj2, isSelfIntersection, maxDomains, cursorHint);
	if (candidates.empty())
		return std::nullopt;

	// STAGE 2 & 3: REFINE AND TRACE THE INTERSECTION CURVE
	std::vector<Vec4f> triedRefined;
	constexpr int maxTraceAttempts = 5;

	for (const auto& candidate : candidates)
	{
		auto refinedParams = RefineIntersectionPoint(obj1, obj2, candidate.params, maxDomains, isSelfIntersection);
		if (!refinedParams)
			continue;

		bool alreadyTried = false;
		for (const auto& prev : triedRefined)
		{
			Vec4f diff = *refinedParams - prev;
			if (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z + diff.w * diff.w < 1e-3f)
			{
				alreadyTried = true;
				break;
			}
		}
		if (alreadyTried)
			continue;

		triedRefined.push_back(*refinedParams);

		if (auto result = TraceIntersectionPath(obj1, obj2, *refinedParams, step, maxDomains, isSelfIntersection))
			return result;

		if (static_cast<int>(triedRefined.size()) >= maxTraceAttempts)
			break;
	}

	return std::nullopt;
}
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
	constexpr int maxDepth = 10;
	if (depth > maxDepth)
		return std::nullopt;

	Vec4f maxDomains(GetMaxU(obj1), GetMaxV(obj1), GetMaxU(obj2), GetMaxV(obj2));

	auto [s_p1, s_du1, s_dv1] = EvaluateParametric(obj1, startParams.x, startParams.y);
	auto [s_p2, s_du2, s_dv2] = EvaluateParametric(obj2, startParams.z, startParams.w);
	Vec3f s_np = Vec3f::cross(s_du1, s_dv1).normalize();
	Vec3f s_nq = Vec3f::cross(s_du2, s_dv2).normalize();
	Vec3f s_rawT = Vec3f::cross(s_np, s_nq);
	float dirSign = (Vec3f::dot(s_rawT, t) >= 0.0f) ? 1.0f : -1.0f;

	Vec4f currentParams = startParams;
	bool converged = false;
	constexpr int maxIterations = 35;
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
			if ((p1 - p0).length_sqr() <= (2.0f * d) * (2.0f * d))
			{
				bool validStep = true;
				for (int dim = 0; dim < 4; dim++)
				{
					if (std::abs(currentParams.f[dim] - startParams.f[dim]) > std::max(maxDomains.f[dim] * 0.5f, 0.5f))
					{
						validStep = false;
						break;
					}
					if (!IsSeamDim(obj1, obj2, currentParams, dim) && (currentParams.f[dim] < 0.0f || currentParams.f[dim] > maxDomains.f[dim]))
					{
						float target = (currentParams.f[dim] < 0.0f) ? 0.0f : maxDomains.f[dim];
						float deltaD = currentParams.f[dim] - startParams.f[dim];
						float tHit = (std::abs(deltaD) > 1e-8f) ? ((target - startParams.f[dim]) / deltaD) : 0.0f;
						Vec4f guess = startParams + (currentParams - startParams) * tHit;
						if (!FindExactEdgePoint(obj1, obj2, guess, dim, target))
						{
							validStep = false;
							break;
						}
					}
				}

				if (validStep)
				{
					Vec3f np1 = Vec3f::cross(du1, dv1).normalize();
					Vec3f nq1 = Vec3f::cross(du2, dv2).normalize();
					Vec3f rawT1 = Vec3f::cross(np1, nq1);
					float rawT1Len = rawT1.length();
					if (depth < maxDepth && rawT1Len >= 0.05f)
					{
						Vec3f tNext = (rawT1 / rawT1Len) * dirSign;
						if (Vec3f::dot(tNext, t) < 0.5f)
						{
							validStep = false;
						}
					}
				}

				if (validStep)
					converged = true;
			}
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

		Vec4f delta = *deltaOpt;
		float maxRelStep = 0.0f;
		for (int dim = 0; dim < 4; dim++)
			maxRelStep = std::max(maxRelStep, std::abs(delta.f[dim]) / std::max(maxDomains.f[dim], 1.0f));
		if (maxRelStep > 0.5f)
			delta *= (0.5f / maxRelStep);

		currentParams += delta;
	}

	if (converged)
		return currentParams;

	return FindIntersectionNextPoint(obj1, obj2, startParams, p0, t, d / 2.0f, depth + 1);
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

bool GeometryMath::IsOnOpenBoundary(SceneObject* obj, float u, float v)
{
	float maxU = GetMaxU(obj);
	float maxV = GetMaxV(obj);
	float edgeEps = std::max(maxU, maxV) * 1e-3f;

	if ((u <= edgeEps || maxU - u <= edgeEps) && !IsSeamConnected(obj, true, v))
		return true;
	if ((v <= edgeEps || maxV - v <= edgeEps) && !IsSeamConnected(obj, false, u))
		return true;
	return false;
}
#pragma endregion
