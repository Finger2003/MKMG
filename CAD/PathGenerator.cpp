#include "pch.h"
#include "PathGenerator.h"
#include "GeometryMath.h"
#include "Intersection.h"
#include "Surface.h"
#include "Torus.h"
#include "../MathLib/Vec2f.h"

using namespace MathLib;

namespace
{
	constexpr float cMmToWorld = 0.05f;
	constexpr float cWorldToMm = 1.0f / cMmToWorld; // 20.0f

	struct SegmentTask
	{
		SceneObject* obj;
		float u0, u1;
		float v0, v1;
	};

	struct CreaseTask
	{
		SceneObject* obj;
		bool alongU; // true: fixed v = fixedParam, vary u; false: fixed u = fixedParam, vary v
		float fixedParam;
		float rangeStart;
		float rangeEnd;
		float maxDomain;
	};

	inline void AtomicMaxFloat(float* target, float val)
	{
		if (std::isnan(val))
			return;

		std::atomic_ref<float> atomicTarget(*target);
		float current = atomicTarget.load(std::memory_order_relaxed);
		while (current < val || std::isnan(current))
		{
			// memory_order_relaxed is sufficient here because each grid cell is updated
			// independently (no inter-variable synchronization required), and OpenMP
			// establishes a full memory barrier at the end of the parallel loop.
			if (atomicTarget.compare_exchange_weak(
				current, val,
				std::memory_order_relaxed,
				std::memory_order_relaxed))
			{
				break;
			}
		}
	}

	float3 SceneToGCodeCoords(const float3& scenePos)
	{
		// Inverse of G-Code mapping in MillingSimulator (X -> x, Y -> -z, Z -> y)
		return { scenePos.x, -scenePos.z, scenePos.y };
	}

	void SplatPointToGrid(
		const Vec3f& posMm,
		float halfX,
		float halfZ,
		float invDx,
		float invDz,
		int divX,
		int divZ,
		std::vector<float>& heightGrid)
	{
		const int cols = divX + 1;
		float fx = (posMm.x + halfX) * invDx;
		float fz = (posMm.z + halfZ) * invDz;

		int ix0 = static_cast<int>(std::floor(fx));
		int ix1 = static_cast<int>(std::ceil(fx));
		int iz0 = static_cast<int>(std::floor(fz));
		int iz1 = static_cast<int>(std::ceil(fz));

		if (ix1 < 0 || ix0 > divX || iz1 < 0 || iz0 > divZ)
			return;

		ix0 = std::clamp(ix0, 0, divX);
		ix1 = std::clamp(ix1, 0, divX);
		iz0 = std::clamp(iz0, 0, divZ);
		iz1 = std::clamp(iz1, 0, divZ);

		for (int iz = iz0; iz <= iz1; iz++)
		{
			size_t rowOffset = static_cast<size_t>(iz) * cols;
			for (int ix = ix0; ix <= ix1; ix++)
			{
				size_t idx = rowOffset + ix;
				if (posMm.y > heightGrid[idx])
				{
					AtomicMaxFloat(&heightGrid[idx], posMm.y);
				}
			}
		}
	}

	std::optional<Vec3f> ComputeOffsetPoint(
		const Vec3f& p,
		const Vec3f& du,
		const Vec3f& dv,
		float effectiveR)
	{
		Vec3f crossN = Vec3f::cross(du, dv);
		float lenSq = crossN.length_sqr();
		if (lenSq <= 1e-12f)
			return std::nullopt;

		Vec3f n = crossN / std::sqrt(lenSq);
		if (n.y < 0.0f)
			n = -n;

		return p * cWorldToMm + n * effectiveR;
	}

	std::optional<Vec3f> EvaluateOffsetPoint(
		SceneObject* obj,
		float u,
		float v,
		float effectiveR)
	{
		auto [p, du, dv] = GeometryMath::EvaluateParametric(obj, u, v);
		return ComputeOffsetPoint(p, du, dv, effectiveR);
	}

	void SplatQuadSubdivided(
		const Vec3f& p00,
		const Vec3f& p10,
		const Vec3f& p01,
		const Vec3f& p11,
		float targetStepMm,
		float halfX,
		float halfZ,
		float invDx,
		float invDz,
		int divX,
		int divZ,
		std::vector<float>& heightGrid)
	{
		float maxEdgeU = std::max((p10 - p00).length(), (p11 - p01).length());
		float maxEdgeV = std::max((p01 - p00).length(), (p11 - p10).length());

		int subU = std::clamp(static_cast<int>(std::ceil(maxEdgeU / targetStepMm)), 1, 32);
		int subV = std::clamp(static_cast<int>(std::ceil(maxEdgeV / targetStepMm)), 1, 32);

		if (subU <= 1 && subV <= 1)
			return;

		for (int iv = 0; iv <= subV; iv++)
		{
			float tv = static_cast<float>(iv) / subV;
			Vec3f left = Vec3f::lerp(p00, p01, tv);
			Vec3f right = Vec3f::lerp(p10, p11, tv);
			for (int iu = 0; iu <= subU; iu++)
			{
				// Skip the 4 exact quad corners splatted in outer loop
				if ((iu == 0 || iu == subU) && (iv == 0 || iv == subV))
					continue;

				float tu = static_cast<float>(iu) / subU;
				Vec3f pt = Vec3f::lerp(left, right, tu);
				SplatPointToGrid(pt, halfX, halfZ, invDx, invDz, divX, divZ, heightGrid);
			}
		}
	}

	void SampleSegmentToCLMap(
		SceneObject* obj,
		float uStart,
		float uEnd,
		float vStart,
		float vEnd,
		float effectiveR,
		float halfX,
		float halfZ,
		float dx,
		float dz,
		int divX,
		int divZ,
		std::vector<float>& clGrid)
	{
		const float invDx = 1.0f / dx;
		const float invDz = 1.0f / dz;
		const float targetStepMm = 1.2f * std::min(dx, dz);

		// Keep Bezier evaluation strictly inside the current segment so C0 seams don't bleed across boundaries
		const float inset = (obj->type == ObjectType::BezierSurface) ? 1e-5f : 0.0f;
		const float uLo = uStart + inset;
		const float uHi = uEnd - inset;
		const float vLo = vStart + inset;
		const float vHi = vEnd - inset;

		// Coarse probe across the offset segment to account for both surface scale and curvature stretching
		float segMaxStepUMm = 1.0f;
		float segMaxStepVMm = 1.0f;
		constexpr int cProbeSamples = 6;
		std::optional<Vec3f> probe[cProbeSamples + 1][cProbeSamples + 1];
		for (int iv = 0; iv <= cProbeSamples; iv++)
		{
			float v = std::lerp(vLo, vHi, static_cast<float>(iv) / cProbeSamples);
			for (int iu = 0; iu <= cProbeSamples; iu++)
			{
				float u = std::lerp(uLo, uHi, static_cast<float>(iu) / cProbeSamples);
				auto [p, du, dv] = GeometryMath::EvaluateParametric(obj, u, v);
				probe[iv][iu] = ComputeOffsetPoint(p, du, dv, effectiveR);

				segMaxStepUMm = std::max(segMaxStepUMm, (uEnd - uStart) * du.length() * cWorldToMm);
				segMaxStepVMm = std::max(segMaxStepVMm, (vEnd - vStart) * dv.length() * cWorldToMm);

				if (iu > 0 && probe[iv][iu] && probe[iv][iu - 1])
				{
					float offsetSpanU = (*probe[iv][iu] - *probe[iv][iu - 1]).length() * cProbeSamples;
					segMaxStepUMm = std::max(segMaxStepUMm, offsetSpanU);
				}
				if (iv > 0 && probe[iv][iu] && probe[iv - 1][iu])
				{
					float offsetSpanV = (*probe[iv][iu] - *probe[iv - 1][iu]).length() * cProbeSamples;
					segMaxStepVMm = std::max(segMaxStepVMm, offsetSpanV);
				}
			}
		}

		// Adaptive step counts for this segment based on offset surface arc length
		const int stepsU = std::clamp(static_cast<int>(std::ceil(segMaxStepUMm / targetStepMm)), 24, 400);
		const int stepsV = std::clamp(static_cast<int>(std::ceil(segMaxStepVMm / targetStepMm)), 24, 400);

		std::vector<std::optional<Vec3f>> prevRow(stepsU + 1);
		std::vector<std::optional<Vec3f>> currRow(stepsU + 1);

		for (int iv = 0; iv <= stepsV; iv++)
		{
			float v = std::lerp(vLo, vHi, static_cast<float>(iv) / stepsV);
			for (int iu = 0; iu <= stepsU; iu++)
			{
				float u = std::lerp(uLo, uHi, static_cast<float>(iu) / stepsU);
				std::optional<Vec3f> s = EvaluateOffsetPoint(obj, u, v, effectiveR);
				currRow[iu] = s;
				if (s)
					SplatPointToGrid(*s, halfX, halfZ, invDx, invDz, divX, divZ, clGrid);

				if (iv > 0 && iu > 0)
				{
					const auto& p00 = prevRow[iu - 1];
					const auto& p10 = prevRow[iu];
					const auto& p01 = currRow[iu - 1];
					const auto& p11 = currRow[iu];

					if (p00 && p10 && p01 && p11)
					{
						SplatQuadSubdivided(
							*p00, *p10,
							*p01, *p11,
							targetStepMm, halfX, halfZ, invDx, invDz, divX, divZ, clGrid);
					}
				}
			}
			std::swap(prevRow, currRow);
		}
	}

	void SampleBezierCreaseToCLMap(
		const CreaseTask& task,
		float effectiveR,
		float halfX,
		float halfZ,
		float dx,
		float dz,
		int divX,
		int divZ,
		std::vector<float>& clGrid)
	{
		const float invDx = 1.0f / dx;
		const float invDz = 1.0f / dz;
		const float targetStepMm = 0.8f * std::min(dx, dz);

		constexpr float cCreaseEps = 1e-4f;
		constexpr float cCreaseCosThreshold = 0.9995f; // ~1.8 deg normal discontinuity

		const float paramSideA = (task.fixedParam == 0.0f)
			? (task.maxDomain - cCreaseEps)
			: (task.fixedParam - cCreaseEps);
		const float paramSideB = task.fixedParam + cCreaseEps;

		// Probe along the seam curve to choose step count
		float maxTangLenMm = 1.0f;
		constexpr int cProbeSamples = 16;
		for (int i = 0; i <= cProbeSamples; i++)
		{
			float tParam = std::lerp(task.rangeStart, task.rangeEnd, static_cast<float>(i) / cProbeSamples);
			float u = task.alongU ? tParam : task.fixedParam;
			float v = task.alongU ? task.fixedParam : tParam;
			auto eval = GeometryMath::EvaluateParametric(task.obj, u, v);
			Vec3f tang = task.alongU ? eval.du : eval.dv;
			maxTangLenMm = std::max(maxTangLenMm, tang.length() * cWorldToMm);
		}

		const int seamSteps = std::clamp(
			static_cast<int>(std::ceil((task.rangeEnd - task.rangeStart) * maxTangLenMm * 1.25f / targetStepMm)),
			64, 600);

		for (int i = 0; i <= seamSteps; i++)
		{
			float tParam = std::lerp(task.rangeStart, task.rangeEnd, static_cast<float>(i) / seamSteps);

			float uA = task.alongU ? tParam : paramSideA;
			float vA = task.alongU ? paramSideA : tParam;
			float uB = task.alongU ? tParam : paramSideB;
			float vB = task.alongU ? paramSideB : tParam;

			auto evalA = GeometryMath::EvaluateParametric(task.obj, uA, vA);
			auto evalB = GeometryMath::EvaluateParametric(task.obj, uB, vB);

			Vec3f crossA = Vec3f::cross(evalA.du, evalA.dv);
			Vec3f crossB = Vec3f::cross(evalB.du, evalB.dv);
			float lenSqA = crossA.length_sqr();
			float lenSqB = crossB.length_sqr();
			if (lenSqA <= 1e-12f || lenSqB <= 1e-12f)
				continue;

			Vec3f nA = crossA / std::sqrt(lenSqA);
			Vec3f nB = crossB / std::sqrt(lenSqB);
			if (nA.y < 0.0f) nA = -nA;
			if (nB.y < 0.0f) nB = -nB;

			float dotN = std::clamp(Vec3f::dot(nA, nB), -1.0f, 1.0f);
			if (dotN >= cCreaseCosThreshold)
				continue;

			// Check convexity across the seam: normals diverge along across-seam direction on convex ridges
			Vec3f acrossA = task.alongU ? evalA.dv : evalA.du;
			Vec3f acrossB = task.alongU ? evalB.dv : evalB.du;
			if (acrossA.length_sqr() > 1e-12f && acrossB.length_sqr() > 1e-12f)
			{
				Vec3f chordDir = acrossA.normalize() + acrossB.normalize();
				if (chordDir.length_sqr() > 1e-4f && Vec3f::dot(nB - nA, chordDir) <= 0.0f)
					continue; // Concave valley: handled automatically by z-buffer max()
			}

			Vec3f pMm = ((evalA.p + evalB.p) * 0.5f) * cWorldToMm;
			float theta = std::acos(dotN);
			int arcSteps = std::clamp(static_cast<int>(std::ceil(theta * effectiveR / targetStepMm)), 2, 128);

			for (int k = 0; k <= arcSteps; k++)
			{
				float s = static_cast<float>(k) / arcSteps;
				Vec3f n = Vec3f::lerp(nA, nB, s);
				float nLenSq = n.length_sqr();
				if (nLenSq <= 1e-10f)
					continue;
				n /= std::sqrt(nLenSq);
				Vec3f cMm = pMm + n * effectiveR;
				SplatPointToGrid(cMm, halfX, halfZ, invDx, invDz, divX, divZ, clGrid);
			}
		}
	}

	std::vector<float> BuildCutterLocationHeightMap(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const RoughingParameters& params)
	{
		const int divX = params.gridDivX;
		const int divZ = params.gridDivZ;
		const int cols = divX + 1;
		const int rows = divZ + 1;
		const float halfX = params.blockSizeX * 0.5f;
		const float halfZ = params.blockSizeZ * 0.5f;
		const float dx = params.blockSizeX / divX;
		const float dz = params.blockSizeZ / divZ;

		const float R = params.cutterDiameter * 0.5f;
		const float effectiveR = R + params.surfaceMargin;
		const float minSafeBaseHeight = params.baseHeight + params.baseMargin;
		const float minSafeBaseCenterY = minSafeBaseHeight + R;

		std::vector<float> clGrid(static_cast<size_t>(cols) * rows, minSafeBaseCenterY);

		std::vector<SegmentTask> tasks;
		tasks.reserve(350);
		std::vector<CreaseTask> creaseTasks;
		creaseTasks.reserve(64);

		for (const auto& obj : sceneObjects)
		{
			if (!obj || obj.get() == params.baseSurface)
				continue;

			if (auto surf = obj->As<Surface>())
			{
				unsigned int segU = surf->GetSegmentsU();
				unsigned int segV = surf->GetSegmentsV();
				if (segU == 0 || segV == 0)
					continue;

				for (unsigned int sv = 0; sv < segV; sv++)
				{
					for (unsigned int su = 0; su < segU; su++)
					{
						tasks.push_back({
							obj.get(),
							static_cast<float>(su), static_cast<float>(su + 1),
							static_cast<float>(sv), static_cast<float>(sv + 1)
							});
					}
				}

				if (obj->type == ObjectType::BezierSurface)
				{
					const float maxU = static_cast<float>(segU);
					const float maxV = static_cast<float>(segV);

					auto isClosedSeam = [&](bool seamInU) -> bool
						{
							constexpr float cSeamEpsSq = 1e-6f;
							const float otherMax = seamInU ? maxV : maxU;
							for (int k = 0; k <= 2; k++)
							{
								float t = otherMax * (static_cast<float>(k) * 0.5f);
								Vec3f a = seamInU
									? GeometryMath::EvaluateParametric(obj.get(), 0.0f, t).p
									: GeometryMath::EvaluateParametric(obj.get(), t, 0.0f).p;
								Vec3f b = seamInU
									? GeometryMath::EvaluateParametric(obj.get(), maxU, t).p
									: GeometryMath::EvaluateParametric(obj.get(), t, maxV).p;
								if ((a - b).length_sqr() >= cSeamEpsSq)
									return false;
							}
							return true;
						};

					const bool closedU = isClosedSeam(true);
					const bool closedV = isClosedSeam(false);

					if (closedU)
						creaseTasks.push_back({ obj.get(), false, 0.0f, 0.0f, maxV, maxU });
					for (unsigned int su = 1; su < segU; su++)
						creaseTasks.push_back({ obj.get(), false, static_cast<float>(su), 0.0f, maxV, maxU });

					if (closedV)
						creaseTasks.push_back({ obj.get(), true, 0.0f, 0.0f, maxU, maxV });
					for (unsigned int sv = 1; sv < segV; sv++)
						creaseTasks.push_back({ obj.get(), true, static_cast<float>(sv), 0.0f, maxU, maxV });
				}
			}
			else if (obj->type == ObjectType::Torus)
			{
				constexpr int cTorusSubPatches = 4;
				for (int sv = 0; sv < cTorusSubPatches; sv++)
				{
					float v0 = static_cast<float>(sv) / cTorusSubPatches;
					float v1 = static_cast<float>(sv + 1) / cTorusSubPatches;
					for (int su = 0; su < cTorusSubPatches; su++)
					{
						float u0 = static_cast<float>(su) / cTorusSubPatches;
						float u1 = static_cast<float>(su + 1) / cTorusSubPatches;
						tasks.push_back({ obj.get(), u0, u1, v0, v1 });
					}
				}
			}
		}

		// 1. Splat offset surfaces P + effectiveR * n directly into the cutter-location (CL) height map
#pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < static_cast<int>(tasks.size()); i++)
		{
			const auto& task = tasks[i];
			SampleSegmentToCLMap(
				task.obj,
				task.u0, task.u1,
				task.v0, task.v1,
				effectiveR,
				halfX, halfZ, dx, dz,
				divX, divZ,
				clGrid);
		}

		// 2. Patch convex C0 creases on Bezier surfaces via normal-cone interpolation
#pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < static_cast<int>(creaseTasks.size()); i++)
		{
			SampleBezierCreaseToCLMap(
				creaseTasks[i],
				effectiveR,
				halfX, halfZ, dx, dz,
				divX, divZ,
				clGrid);
		}

		// 3. Apply downward slope limiter so no steep cliff on Z_CL can cause a near-vertical plunge
		const float maxSlope = std::max(0.5f, params.maxPlungeSlope);
		const float maxDropX = dx * maxSlope;
		const float maxDropZ = dz * maxSlope;

#pragma omp parallel for schedule(static)
		for (int iz = 0; iz <= divZ; iz++)
		{
			size_t rowOffset = static_cast<size_t>(iz) * cols;
			for (int ix = 1; ix <= divX; ix++)
				clGrid[rowOffset + ix] = std::max(clGrid[rowOffset + ix], clGrid[rowOffset + ix - 1] - maxDropX);
			for (int ix = divX - 1; ix >= 0; ix--)
				clGrid[rowOffset + ix] = std::max(clGrid[rowOffset + ix], clGrid[rowOffset + ix + 1] - maxDropX);
		}

#pragma omp parallel for schedule(static)
		for (int ix = 0; ix <= divX; ix++)
		{
			for (int iz = 1; iz <= divZ; iz++)
				clGrid[static_cast<size_t>(iz) * cols + ix] = std::max(clGrid[static_cast<size_t>(iz) * cols + ix], clGrid[static_cast<size_t>(iz - 1) * cols + ix] - maxDropZ);
			for (int iz = divZ - 1; iz >= 0; iz--)
				clGrid[static_cast<size_t>(iz) * cols + ix] = std::max(clGrid[static_cast<size_t>(iz) * cols + ix], clGrid[static_cast<size_t>(iz + 1) * cols + ix] - maxDropZ);
		}

		// 4. Convert cutter-sphere center height map to cutter-tip height map
#pragma omp parallel for schedule(static)
		for (int idx = 0; idx < static_cast<int>(clGrid.size()); idx++)
			clGrid[idx] -= R;

		return clGrid;
	}

	std::vector<float3> SimplifyScanlineSafe(const std::vector<float3>& rawPts, float tolerance)
	{
		if (rawPts.size() <= 2)
			return rawPts;

		const int n = static_cast<int>(rawPts.size());
		std::vector<bool> keep(n, false);
		keep.front() = true;
		keep.back() = true;

		std::vector<std::pair<int, int>> stack;
		stack.reserve(n);
		stack.push_back({ 0, n - 1 });

		constexpr float cUndercutEps = 1e-4f;

		while (!stack.empty())
		{
			auto [iStart, iEnd] = stack.back();
			stack.pop_back();

			if (iEnd - iStart <= 1)
				continue;

			const float3& pStart = rawPts[iStart];
			const float3& pEnd = rawPts[iEnd];
			const float invSpan = 1.0f / static_cast<float>(iEnd - iStart);

			float maxAbsErr = 0.0f;
			int maxErrIdx = -1;

			float worstUndercut = 0.0f; // most negative (interpY - rawPts[i].y)
			int worstUndercutIdx = -1;

			for (int i = iStart + 1; i < iEnd; i++)
			{
				float t = static_cast<float>(i - iStart) * invSpan;
				float interpY = pStart.y + t * (pEnd.y - pStart.y);
				float diff = interpY - rawPts[i].y;

				if (diff < -cUndercutEps && diff < worstUndercut)
				{
					worstUndercut = diff;
					worstUndercutIdx = i;
				}

				float absErr = std::abs(diff);
				if (absErr > maxAbsErr)
				{
					maxAbsErr = absErr;
					maxErrIdx = i;
				}
			}

			int splitIdx = -1;
			if (worstUndercutIdx != -1)
				splitIdx = worstUndercutIdx;
			else if (maxAbsErr > tolerance)
				splitIdx = maxErrIdx;

			if (splitIdx != -1)
			{
				keep[splitIdx] = true;
				stack.push_back({ iStart, splitIdx });
				stack.push_back({ splitIdx, iEnd });
			}
		}

		std::vector<float3> simplified;
		simplified.reserve(n);
		for (int i = 0; i < n; i++)
		{
			if (keep[i])
				simplified.push_back(rawPts[i]);
		}
		return simplified;
	}

	// =========================================================================
	// 2D Geometry & Planar Outer-Envelope Helpers for Base Finishing (F10)
	// =========================================================================
	float ComputeSignedAreaXZ(const std::vector<Vec2f>& poly)
	{
		const size_t n = poly.size();
		if (n < 3)
			return 0.0f;

		double area = 0.0;
		for (size_t i = 0, j = n - 1; i < n; j = i++)
		{
			area += static_cast<double>(Vec2f::cross(poly[j], poly[i]));
		}
		return static_cast<float>(0.5 * area);
	}

	bool IsPointInPolygonXZ(const Vec2f& pt, const std::vector<Vec2f>& poly)
	{
		bool inside = false;
		const size_t n = poly.size();
		for (size_t i = 0, j = n - 1; i < n; j = i++)
		{
			const Vec2f& a = poly[i];
			const Vec2f& b = poly[j];
			if ((a.z > pt.z) != (b.z > pt.z))
			{
				if ((b.z > a.z) == (Vec2f::cross(b - a, pt - a) > 0.0f))
					inside = !inside;
			}
		}
		return inside;
	}

	std::vector<Vec2f> CleanAndSimplifyLoop2D(const std::vector<Vec2f>& loop, float minEdgeLen = 0.05f)
	{
		std::vector<Vec2f> cleaned;
		cleaned.reserve(loop.size());
		const float minLenSq = minEdgeLen * minEdgeLen;

		for (const auto& pt : loop)
		{
			if (cleaned.empty() || (pt - cleaned.back()).length_sqr() >= minLenSq)
				cleaned.push_back(pt);
		}
		while (cleaned.size() > 2 && (cleaned.back() - cleaned.front()).length_sqr() < minLenSq)
			cleaned.pop_back();

		if (ComputeSignedAreaXZ(cleaned) < 0.0f)
			std::reverse(cleaned.begin(), cleaned.end());

		return cleaned;
	}

	// Extracts outer boundaries reachable from outside (removes self-intersections, overlaps, and enclosed pockets)
	std::vector<std::vector<Vec2f>> ExtractOuterEnvelopes2D(const std::vector<std::vector<Vec2f>>& rawLoops)
	{
		struct Seg2D
		{
			Vec2f a, b;
			float minX, maxX, minZ, maxZ;
		};

		std::vector<Seg2D> segments;
		for (const auto& raw : rawLoops)
		{
			auto loop = CleanAndSimplifyLoop2D(raw, 0.02f);
			float area = std::abs(ComputeSignedAreaXZ(loop));
			if (loop.size() < 3 || area < 1e-2f)
				continue;

			const size_t n = loop.size();
			for (size_t i = 0, j = n - 1; i < n; j = i++)
			{
				const Vec2f& a = loop[j];
				const Vec2f& b = loop[i];
				auto [minX, maxX] = std::minmax(a.x, b.x);
				auto [minZ, maxZ] = std::minmax(a.z, b.z);
				segments.push_back({
					a, b,
					minX, maxX,
					minZ, maxZ
					});
			}
		}

		if (segments.empty())
			return {};

		std::vector<std::vector<float>> splitParams(segments.size(), { 0.0f, 1.0f });
		constexpr float cSegEps = 1e-5f;

		for (size_t i = 0; i < segments.size(); i++)
		{
			const auto& s1 = segments[i];
			Vec2f d1 = s1.b - s1.a;
			for (size_t j = i + 1; j < segments.size(); j++)
			{
				const auto& s2 = segments[j];
				if (s1.maxX < s2.minX - cSegEps || s2.maxX < s1.minX - cSegEps ||
					s1.maxZ < s2.minZ - cSegEps || s2.maxZ < s1.minZ - cSegEps)
				{
					continue;
				}

				Vec2f d2 = s2.b - s2.a;
				float denom = Vec2f::cross(d1, d2);
				Vec2f diff = s2.a - s1.a;

				if (std::abs(denom) > 1e-8f)
				{
					float t1 = Vec2f::cross(diff, d2) / denom;
					float t2 = Vec2f::cross(diff, d1) / denom;
					if (t1 >= -cSegEps && t1 <= 1.0f + cSegEps &&
						t2 >= -cSegEps && t2 <= 1.0f + cSegEps)
					{
						splitParams[i].push_back(std::clamp(t1, 0.0f, 1.0f));
						splitParams[j].push_back(std::clamp(t2, 0.0f, 1.0f));
					}
				}
				else if (std::abs(Vec2f::cross(diff, d1)) <= 1e-5f)
				{
					// Collinear overlap: project endpoints
					float lenSq1 = d1.length_sqr();
					float lenSq2 = d2.length_sqr();
					if (lenSq1 > 1e-9f)
					{
						float pA = Vec2f::dot(s2.a - s1.a, d1) / lenSq1;
						float pB = Vec2f::dot(s2.b - s1.a, d1) / lenSq1;
						if (pA > 0.0f && pA < 1.0f) splitParams[i].push_back(pA);
						if (pB > 0.0f && pB < 1.0f) splitParams[i].push_back(pB);
					}
					if (lenSq2 > 1e-9f)
					{
						float qA = Vec2f::dot(s1.a - s2.a, d2) / lenSq2;
						float qB = Vec2f::dot(s1.b - s2.a, d2) / lenSq2;
						if (qA > 0.0f && qA < 1.0f) splitParams[j].push_back(qA);
						if (qB > 0.0f && qB < 1.0f) splitParams[j].push_back(qB);
					}
				}
			}
		}

		// Vertex welding & undirected edge collection
		std::vector<Vec2f> vertices;
		constexpr float cWeldEps = 1e-3f;
		constexpr float cWeldEpsSq = cWeldEps * cWeldEps;

		auto getOrAddVertex = [&](const Vec2f& p) -> int
			{
				for (size_t k = 0; k < vertices.size(); k++)
				{
					if ((vertices[k] - p).length_sqr() <= cWeldEpsSq)
						return static_cast<int>(k);
				}
				vertices.push_back(p);
				return static_cast<int>(vertices.size() - 1);
			};

		std::vector<std::pair<int, int>> rawEdges;
		for (size_t i = 0; i < segments.size(); i++)
		{
			auto& ts = splitParams[i];
			std::sort(ts.begin(), ts.end());
			const Vec2f d = segments[i].b - segments[i].a;

			int prevV = getOrAddVertex(segments[i].a);
			for (size_t k = 1; k + 1 < ts.size(); k++)
			{
				if (ts[k] - ts[k - 1] < 1e-5f || 1.0f - ts[k] < 1e-5f)
					continue;
				int currV = getOrAddVertex(segments[i].a + d * ts[k]);
				if (currV != prevV)
				{
					int u = std::min(prevV, currV);
					int v = std::max(prevV, currV);
					rawEdges.emplace_back(u, v);
				}
				prevV = currV;
			}
			int endV = getOrAddVertex(segments[i].b);
			if (endV != prevV)
			{
				int u = std::min(prevV, endV);
				int v = std::max(prevV, endV);
				rawEdges.emplace_back(u, v);
			}
		}

		std::sort(rawEdges.begin(), rawEdges.end());
		rawEdges.erase(std::unique(rawEdges.begin(), rawEdges.end()), rawEdges.end());

		// Build adjacency list and prune degree-1 whiskers iteratively
		std::vector<std::vector<int>> adj(vertices.size());
		for (const auto& [u, v] : rawEdges)
		{
			adj[u].push_back(v);
			adj[v].push_back(u);
		}

		std::vector<bool> vertexRemoved(vertices.size(), false);
		std::queue<int> pruneQueue;
		for (size_t v = 0; v < vertices.size(); v++)
		{
			if (adj[v].size() <= 1)
			{
				vertexRemoved[v] = true;
				pruneQueue.push(static_cast<int>(v));
			}
		}
		while (!pruneQueue.empty())
		{
			int u = pruneQueue.front();
			pruneQueue.pop();
			for (int nb : adj[u])
			{
				if (vertexRemoved[nb])
					continue;
				auto& nbAdj = adj[nb];
				nbAdj.erase(std::remove(nbAdj.begin(), nbAdj.end(), u), nbAdj.end());
				if (nbAdj.size() <= 1)
				{
					vertexRemoved[nb] = true;
					pruneQueue.push(nb);
				}
			}
			adj[u].clear();
		}

		size_t removedCount = static_cast<size_t>(std::count(vertexRemoved.begin(), vertexRemoved.end(), true));

		// Connected components -> extract outer envelope of each component via outer-face walk
		std::vector<bool> compVisited(vertices.size(), false);
		std::vector<std::vector<Vec2f>> outerEnvelopes;
		std::vector<float> outerAreas;

		auto isSmallerAngleCCW = [](const Vec2f& refDir, const Vec2f& a, const Vec2f& b) -> bool
			{
				const float refLenSq = refDir.length_sqr();
				auto inFirstHalf = [&](const Vec2f& v) -> bool
					{
						float c = Vec2f::cross(refDir, v);
						float epsCrossSq = 1e-12f * refLenSq * v.length_sqr();
						if (c * c <= epsCrossSq)
							return Vec2f::dot(refDir, v) < 0.0f; // 180 deg -> first half; 0 deg -> second half (2*pi)
						return c > 0.0f;
					};

				bool firstA = inFirstHalf(a);
				bool firstB = inFirstHalf(b);
				if (firstA != firstB)
					return firstA;

				return Vec2f::cross(a, b) > 0.0f;
			};

		for (size_t startV = 0; startV < vertices.size(); startV++)
		{
			if (vertexRemoved[startV] || compVisited[startV] || adj[startV].empty())
				continue;

			// BFS connected component
			std::vector<int> compVerts;
			std::queue<int> bfs;
			compVisited[startV] = true;
			bfs.push(static_cast<int>(startV));
			while (!bfs.empty())
			{
				int u = bfs.front();
				bfs.pop();
				compVerts.push_back(u);
				for (int nb : adj[u])
				{
					if (!compVisited[nb])
					{
						compVisited[nb] = true;
						bfs.push(nb);
					}
				}
			}

			// Find bottom-left-most vertex (guaranteed to lie on convex hull -> outer face)
			int hullV = compVerts.front();
			for (int v : compVerts)
			{
				if (vertices[v].x < vertices[hullV].x - 1e-6f ||
					(std::abs(vertices[v].x - vertices[hullV].x) <= 1e-6f && vertices[v].z < vertices[hullV].z))
				{
					hullV = v;
				}
			}

			// Start along the edge with smallest angle from -Z/right (keeps outside to the right -> CCW walk)
			const Vec2f leftRef{ -1.0f, 0.0f };
			int firstNext = -1;
			Vec2f bestFirstDir{};
			for (int nb : adj[hullV])
			{
				Vec2f d = vertices[nb] - vertices[hullV];
				if (firstNext == -1 || isSmallerAngleCCW(leftRef, d, bestFirstDir))
				{
					bestFirstDir = d;
					firstNext = nb;
				}
			}
			if (firstNext == -1)
				continue;

			std::vector<Vec2f> boundary;
			int prevV = hullV;
			int currV = firstNext;
			boundary.push_back(vertices[hullV]);

			const size_t maxWalkSteps = compVerts.size() * 4 + 16;
			for (size_t step = 0; step < maxWalkSteps; step++)
			{
				if (currV == hullV)
					break;

				boundary.push_back(vertices[currV]);

				Vec2f backDir = vertices[prevV] - vertices[currV];
				int bestNext = -1;
				Vec2f bestFwdDir{};
				for (int nb : adj[currV])
				{
					if (nb == prevV && adj[currV].size() > 1)
						continue;
					Vec2f fwdDir = vertices[nb] - vertices[currV];
					if (bestNext == -1 || isSmallerAngleCCW(backDir, fwdDir, bestFwdDir))
					{
						bestFwdDir = fwdDir;
						bestNext = nb;
					}
				}

				if (bestNext == -1)
					break;

				prevV = currV;
				currV = bestNext;
			}

			boundary = CleanAndSimplifyLoop2D(boundary, 0.05f);
			float bArea = std::abs(ComputeSignedAreaXZ(boundary));
			if (boundary.size() >= 3 && bArea > 1e-1f)
			{
				outerEnvelopes.push_back(std::move(boundary));
				outerAreas.push_back(bArea);
			}
		}

		// Filter out loops that are completely enclosed inside another outer loop (e.g. mast inside hull)
		std::vector<std::vector<Vec2f>> finalOuter;
		for (size_t i = 0; i < outerEnvelopes.size(); i++)
		{
			bool enclosed = false;
			const Vec2f& probePt = outerEnvelopes[i].front();
			for (size_t j = 0; j < outerEnvelopes.size(); j++)
			{
				if (i == j)
					continue;
				if (outerAreas[j] > outerAreas[i] &&
					IsPointInPolygonXZ(probePt, outerEnvelopes[j]))
				{
					enclosed = true;
					break;
				}
			}
			if (!enclosed)
				finalOuter.push_back(std::move(outerEnvelopes[i]));
		}

		return finalOuter;
	}

	std::vector<std::pair<uint32_t, std::vector<float3>>> LoadBaseCursorSeedsFromJson(const std::string& filepath)
	{
		std::vector<std::pair<uint32_t, std::vector<float3>>> seedsById;
		if (filepath.empty())
			return seedsById;

		std::ifstream in(filepath);
		if (!in.is_open())
			return seedsById;

		try
		{
			nlohmann::json j;
			in >> j;

			if (!j.contains("surfaces") || !j["surfaces"].is_array())
				return seedsById;

			for (const auto& item : j["surfaces"])
			{
				if (!item.contains("id") || !item["id"].is_number_integer())
					continue;

				const nlohmann::json* seedsArr = nullptr;
				if (item.contains("baseCursorSeeds") && item["baseCursorSeeds"].is_array())
					seedsArr = &item["baseCursorSeeds"];
				else if (item.contains("cursorSeeds") && item["cursorSeeds"].is_array())
					seedsArr = &item["cursorSeeds"];

				if (!seedsArr || seedsArr->empty())
					continue;

				auto seeds = seedsArr->get<std::vector<float3>>();
				if (!seeds.empty())
					seedsById.emplace_back(item["id"].get<uint32_t>(), std::move(seeds));
			}
		}
		catch (...)
		{
			return {};
		}

		return seedsById;
	}

	struct OpenCLPiece2D
	{
		std::vector<Vec2f> clPts;
		Vec2f baseStart{};
		Vec2f baseEnd{};
	};

	void AppendCornerArc2D(
		std::vector<Vec2f>& outLoop,
		const Vec2f& centerMm,
		const Vec2f& clFrom,
		const Vec2f& clTo,
		float offsetMm)
	{
		Vec2f vFrom = clFrom - centerMm;
		Vec2f vTo = clTo - centerMm;
		float lenFromSq = vFrom.length_sqr();
		float lenToSq = vTo.length_sqr();
		if (lenFromSq <= 1e-10f || lenToSq <= 1e-10f)
			return;

		Vec2f hFrom = vFrom / std::sqrt(lenFromSq);
		Vec2f hTo = vTo / std::sqrt(lenToSq);
		float dotH = std::clamp(Vec2f::dot(hFrom, hTo), -1.0f, 1.0f);
		if (dotH >= 0.996f)
			return;

		float theta = std::acos(dotH);
		int arcSteps = std::clamp(static_cast<int>(std::ceil(theta * offsetMm / 0.5f)), 2, 32);
		float sign = (Vec2f::cross(hFrom, hTo) >= 0.0f) ? 1.0f : -1.0f;
		Vec2f perp{ -hFrom.z * sign, hFrom.x * sign };

		for (int k = 1; k < arcSteps; k++)
		{
			float a = theta * (static_cast<float>(k) / arcSteps);
			Vec2f h = hFrom * std::cos(a) + perp * std::sin(a);
			outLoop.push_back(centerMm + h * offsetMm);
		}
	}

	std::vector<Vec2f> StitchCLPieces2D(
		std::vector<OpenCLPiece2D> pieces,
		float offsetMm)
	{
		if (pieces.empty())
			return {};

		std::vector<OpenCLPiece2D> ordered;
		ordered.reserve(pieces.size());
		std::vector<bool> used(pieces.size(), false);
		ordered.push_back(std::move(pieces[0]));
		used[0] = true;

		for (size_t step = 1; step < pieces.size(); step++)
		{
			const Vec2f& currEndBase = ordered.back().baseEnd;
			size_t bestIdx = pieces.size();
			bool bestReverse = false;
			float bestDistSq = std::numeric_limits<float>::max();

			for (size_t i = 0; i < pieces.size(); i++)
			{
				if (used[i])
					continue;
				float dFront = (pieces[i].baseStart - currEndBase).length_sqr();
				if (dFront < bestDistSq)
				{
					bestDistSq = dFront;
					bestIdx = i;
					bestReverse = false;
				}
				float dBack = (pieces[i].baseEnd - currEndBase).length_sqr();
				if (dBack < bestDistSq)
				{
					bestDistSq = dBack;
					bestIdx = i;
					bestReverse = true;
				}
			}

			if (bestIdx >= pieces.size())
				break;

			used[bestIdx] = true;
			if (bestReverse)
			{
				std::reverse(pieces[bestIdx].clPts.begin(), pieces[bestIdx].clPts.end());
				std::swap(pieces[bestIdx].baseStart, pieces[bestIdx].baseEnd);
			}
			ordered.push_back(std::move(pieces[bestIdx]));
		}

		std::vector<Vec2f> stitchedCL;
		const size_t kCount = ordered.size();
		for (size_t i = 0; i < kCount; i++)
		{
			const auto& curr = ordered[i];
			const auto& next = ordered[(i + 1) % kCount];

			stitchedCL.insert(stitchedCL.end(), curr.clPts.begin(), curr.clPts.end());

			Vec2f center = (curr.baseEnd + next.baseStart) * 0.5f;
			AppendCornerArc2D(stitchedCL, center, curr.clPts.back(), next.clPts.front(), offsetMm);
		}

		return stitchedCL;
	}

	std::vector<std::vector<Vec2f>> CollectHorizontalCLContours(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		SceneObject* baseSurface,
		float yBaseWorld,
		float offsetMm,
		const GeometryMath::HorizontalPlaneSurface& horizPlane,
		const std::vector<std::pair<uint32_t, std::vector<float3>>>& seedsById)
	{
		const float offsetWorld = offsetMm * cMmToWorld;
		const float stepWorld = 0.5f * cMmToWorld; // 0.5 mm intersection step

		std::vector<std::vector<Vec2f>> loops;
		for (const auto& obj : sceneObjects)
		{
			if (!obj || obj.get() == baseSurface)
				continue;
			if (!obj->IsA(ObjectType::Surface) && obj->type != ObjectType::Torus)
				continue;

			float maxU = 1.0f, maxV = 1.0f;
			if (auto surf = obj->As<Surface>())
			{
				maxU = static_cast<float>(surf->GetSegmentsU());
				maxV = static_cast<float>(surf->GetSegmentsV());
			}
			Vec4f maxDomains(maxU, maxV, 1.0f, 1.0f);
			GeometryMath::OffsetSurface clSurf(obj.get(), offsetWorld, true);

			auto seedIt = std::find_if(seedsById.begin(), seedsById.end(),
				[id = obj->m_id](const auto& entry) { return entry.first == id; });
			if (seedIt != seedsById.end() && !seedIt->second.empty())
			{
				const auto& seeds = seedIt->second;
				std::vector<OpenCLPiece2D> pieces;
				pieces.reserve(seeds.size());
				for (const auto& seed : seeds)
				{
					auto interOpt = GeometryMath::CalculateIntersection(
						clSurf, horizPlane, false, maxDomains, stepWorld, seed, true);
					if (interOpt && interOpt->points.size() >= 2)
					{
						OpenCLPiece2D piece;
						piece.clPts.reserve(interOpt->points.size());
						for (const auto& p : interOpt->points)
							piece.clPts.push_back({ p.x * cWorldToMm, p.z * cWorldToMm });

						auto evalStart = GeometryMath::EvaluateParametric(obj.get(), interOpt->params.front().x, interOpt->params.front().y);
						auto evalEnd = GeometryMath::EvaluateParametric(obj.get(), interOpt->params.back().x, interOpt->params.back().y);
						piece.baseStart = { evalStart.p.x * cWorldToMm, evalStart.p.z * cWorldToMm };
						piece.baseEnd = { evalEnd.p.x * cWorldToMm, evalEnd.p.z * cWorldToMm };
						pieces.push_back(std::move(piece));
					}
				}

				if (!pieces.empty())
				{
					std::vector<Vec2f> loop2D = StitchCLPieces2D(std::move(pieces), offsetMm);
					if (loop2D.size() >= 3)
					{
						loops.push_back(std::move(loop2D));
						continue;
					}
				}
			}

			auto interOpt = GeometryMath::CalculateIntersection(
				clSurf, horizPlane, false, maxDomains, stepWorld, std::nullopt);

			if (interOpt && interOpt->points.size() >= 3)
			{
				std::vector<Vec2f> loop2D;
				loop2D.reserve(interOpt->points.size());
				for (const auto& p : interOpt->points)
					loop2D.push_back({ p.x * cWorldToMm, p.z * cWorldToMm });

				loops.push_back(std::move(loop2D));
			}
		}

		return ExtractOuterEnvelopes2D(loops);
	}

	float PointToSegmentDistSq2D(const Vec2f& p, const Vec2f& a, const Vec2f& b)
	{
		Vec2f ab = b - a;
		float lenSq = ab.length_sqr();
		if (lenSq < 1e-12f)
			return (p - a).length_sqr();

		float t = std::clamp(Vec2f::dot(p - a, ab) / lenSq, 0.0f, 1.0f);
		Vec2f proj = a + ab * t;
		return (p - proj).length_sqr();
	}

	std::vector<Vec2f> SimplifyPolyline2D(const std::vector<Vec2f>& rawPts, float tolerance)
	{
		if (rawPts.size() <= 2 || tolerance <= 0.0f)
			return rawPts;

		const int n = static_cast<int>(rawPts.size());
		std::vector<bool> keep(n, false);
		keep.front() = true;
		keep.back() = true;

		std::vector<std::pair<int, int>> stack;
		stack.reserve(n);
		stack.push_back({ 0, n - 1 });

		const float tolSq = tolerance * tolerance;

		while (!stack.empty())
		{
			auto [iStart, iEnd] = stack.back();
			stack.pop_back();

			if (iEnd - iStart <= 1)
				continue;

			const Vec2f& pStart = rawPts[iStart];
			const Vec2f& pEnd = rawPts[iEnd];

			float maxDistSq = -1.0f;
			int maxIdx = -1;

			for (int i = iStart + 1; i < iEnd; i++)
			{
				float dSq = PointToSegmentDistSq2D(rawPts[i], pStart, pEnd);
				if (dSq > maxDistSq)
				{
					maxDistSq = dSq;
					maxIdx = i;
				}
			}

			if (maxDistSq > tolSq && maxIdx != -1)
			{
				keep[maxIdx] = true;
				stack.push_back({ iStart, maxIdx });
				stack.push_back({ maxIdx, iEnd });
			}
		}

		std::vector<Vec2f> simplified;
		simplified.reserve(n);
		for (int i = 0; i < n; i++)
		{
			if (keep[i])
				simplified.push_back(rawPts[i]);
		}
		return simplified;
	}

	struct HorizontalHit2D
	{
		float x = 0.0f;
		size_t edgeIdx = 0;
		float t = 0.0f;
		Vec2f pt{};
	};

	std::vector<HorizontalHit2D> IntersectHorizontalWithPolygon2D(float zp, const std::vector<Vec2f>& poly)
	{
		std::vector<HorizontalHit2D> hits;
		const size_t n = poly.size();
		if (n < 3)
			return hits;

		for (size_t i = 0; i < n; i++)
		{
			const Vec2f& a = poly[i];
			const Vec2f& b = poly[(i + 1) % n];
			if ((a.z <= zp && zp < b.z) || (b.z <= zp && zp < a.z))
			{
				float denom = b.z - a.z;
				if (std::abs(denom) > 1e-9f)
				{
					float t = (zp - a.z) / denom;
					float x = a.x + t * (b.x - a.x);
					hits.push_back({ x, i, t,{ x, zp } });
				}
			}
		}

		std::sort(hits.begin(), hits.end(), [](const HorizontalHit2D& a, const HorizontalHit2D& b) {
			return a.x < b.x;
			});
		return hits;
	}

	float ComputePolylineLength2D(const std::vector<Vec2f>& pts)
	{
		float len = 0.0f;
		for (size_t i = 1; i < pts.size(); i++)
			len += (pts[i] - pts[i - 1]).length();
		return len;
	}

	std::vector<Vec2f> ExtractShorterArc2D(
		const std::vector<Vec2f>& poly,
		const HorizontalHit2D& hitA,
		const HorizontalHit2D& hitB)
	{
		const size_t n = poly.size();
		if (n == 0)
			return {};

		std::vector<Vec2f> fwd;
		fwd.push_back(hitA.pt);
		if (hitA.edgeIdx == hitB.edgeIdx && hitB.t >= hitA.t)
		{
			fwd.push_back(hitB.pt);
		}
		else
		{
			size_t i = (hitA.edgeIdx + 1) % n;
			const size_t stop = (hitB.edgeIdx + 1) % n;
			size_t guard = 0;
			while (i != stop && guard++ <= n)
			{
				fwd.push_back(poly[i]);
				i = (i + 1) % n;
			}
			fwd.push_back(hitB.pt);
		}

		std::vector<Vec2f> bwd;
		bwd.push_back(hitA.pt);
		if (hitA.edgeIdx == hitB.edgeIdx && hitB.t <= hitA.t)
		{
			bwd.push_back(hitB.pt);
		}
		else
		{
			size_t i = hitA.edgeIdx;
			size_t guard = 0;
			while (i != hitB.edgeIdx && guard++ <= n)
			{
				bwd.push_back(poly[i]);
				i = (i + n - 1) % n;
			}
			bwd.push_back(hitB.pt);
		}

		return (ComputePolylineLength2D(fwd) <= ComputePolylineLength2D(bwd)) ? fwd : bwd;
	}

	struct BaseWaypointWriter
	{
		std::vector<float3>& waypoints;
		float baseY = 0.0f;

		void Append(float x, float y, float z)
		{
			float3 pt(x, y, z);
			if (waypoints.empty() || (pt.ToVec3f() - waypoints.back().ToVec3f()).length_sqr() > 1e-6f)
				waypoints.push_back(pt);
		}

		void AppendBase(float x, float z)
		{
			Append(x, baseY, z);
		}
	};

	void TraceSideZigzagPasses(
		BaseWaypointWriter& writer,
		const std::vector<Vec2f>& zigContour,
		const std::vector<float>& zList,
		bool isRightSide,
		float outX,
		float simplifyTol)
	{
		for (size_t k = 0; k + 1 < zList.size(); k += 2)
		{
			float zA = zList[k];
			float zB = zList[k + 1];
			auto hitsA = IntersectHorizontalWithPolygon2D(zA, zigContour);
			auto hitsB = IntersectHorizontalWithPolygon2D(zB, zigContour);
			if (hitsA.empty() || hitsB.empty())
				continue;

			const HorizontalHit2D& hitA = isRightSide ? hitsA.back() : hitsA.front();
			const HorizontalHit2D& hitB = isRightSide ? hitsB.back() : hitsB.front();

			writer.AppendBase(outX, zA);
			writer.AppendBase(hitA.pt.x, hitA.pt.z);

			std::vector<Vec2f> simpArc = SimplifyPolyline2D(ExtractShorterArc2D(zigContour, hitA, hitB), simplifyTol);
			for (size_t m = 1; m < simpArc.size(); m++)
				writer.AppendBase(simpArc[m].x, simpArc[m].z);

			writer.AppendBase(outX, zB);
		}

		if (zList.size() % 2 == 1)
		{
			float zLast = zList.back();
			auto hitsLast = IntersectHorizontalWithPolygon2D(zLast, zigContour);
			if (!hitsLast.empty())
			{
				const HorizontalHit2D& hLast = isRightSide ? hitsLast.back() : hitsLast.front();
				writer.AppendBase(outX, zLast);
				writer.AppendBase(hLast.pt.x, zLast);
				writer.AppendBase(outX, zLast);
			}
		}
	}

	std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>> CollectOpenBaysAtZ(
		const std::vector<Vec2f>& zigContour,
		float zp)
	{
		std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>> bays;
		auto hits = IntersectHorizontalWithPolygon2D(zp, zigContour);
		if (hits.size() >= 4)
		{
			for (size_t m = 1; m + 1 < hits.size(); m += 2)
				bays.emplace_back(hits[m], hits[m + 1]);
		}
		return bays;
	}

	void AppendOpenBaysAlongLine(
		BaseWaypointWriter& writer,
		const std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>>& bays,
		float zp,
		bool leftToRight)
	{
		if (bays.empty())
			return;

		auto emitBay = [&](const HorizontalHit2D& enterHit, const HorizontalHit2D& exitHit)
		{
			writer.AppendBase(enterHit.pt.x, zp);
			writer.AppendBase(enterHit.pt.x, enterHit.pt.z);
			writer.AppendBase(exitHit.pt.x, exitHit.pt.z);
			writer.AppendBase(exitHit.pt.x, zp);
		};

		if (leftToRight)
		{
			for (const auto& [hLeft, hRight] : bays)
				emitBay(hRight, hLeft);
		}
		else
		{
			for (auto it = bays.rbegin(); it != bays.rend(); ++it)
				emitBay(it->first, it->second);
		}
	}

	void AppendBottomPassesAndBays(
		BaseWaypointWriter& writer,
		const std::vector<float>& botZ,
		const std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>>& openBays,
		bool currRight,
		float outsideLeftX,
		float outsideRightX,
		float halfZ,
		float cutterR)
	{
		if (!botZ.empty())
		{
			bool botLtr = !currRight;
			for (size_t idxB = 0; idxB < botZ.size(); idxB++)
			{
				float zp = botZ[idxB];
				float startX = botLtr ? outsideLeftX : outsideRightX;
				float endX = botLtr ? outsideRightX : outsideLeftX;

				writer.AppendBase(startX, zp);
				if (idxB == 0)
					AppendOpenBaysAlongLine(writer, openBays, zp, botLtr);
				writer.AppendBase(endX, zp);
				botLtr = !botLtr;
			}

			// If botZ had an even count, step outside the bottom edge to switch sides without backtracking
			if (botZ.size() % 2 == 0)
			{
				float outsideBotZ = std::max(halfZ, botZ.back()) + cutterR + 2.0f;
				writer.AppendBase(writer.waypoints.back().x, outsideBotZ);
				writer.AppendBase(currRight ? outsideLeftX : outsideRightX, outsideBotZ);
			}
			return;
		}

		float outsideBotZ = halfZ + cutterR + 2.0f;
		writer.AppendBase(writer.waypoints.back().x, outsideBotZ);
		AppendOpenBaysAlongLine(writer, openBays, outsideBotZ, !currRight);
		writer.AppendBase(currRight ? outsideLeftX : outsideRightX, outsideBotZ);
	}

	void AppendContourPass(
		BaseWaypointWriter& writer,
		const std::vector<Vec2f>& clContour,
		float halfZ,
		float cutterR,
		float simplifyTol)
	{
		if (clContour.size() < 3)
			return;

		std::vector<Vec2f> closedContour = clContour;
		closedContour.push_back(clContour.front());
		std::vector<Vec2f> simplifiedContour = SimplifyPolyline2D(closedContour, simplifyTol);
		if (simplifiedContour.size() < 3)
			return;

		const float3 endPt = writer.waypoints.back();
		const Vec2f& apex = simplifiedContour.front();
		writer.AppendBase(endPt.x, apex.z);
		for (const auto& pt : simplifiedContour)
			writer.AppendBase(pt.x, pt.z);

		float outsideTopZ = std::min(-halfZ, apex.z) - cutterR - 2.0f;
		writer.AppendBase(apex.x, outsideTopZ);
	}

	// =========================================================================
	// Surface Finishing & Pencil Milling Helpers (K08)
	// =========================================================================

	struct FinishingSurfaceSpec
	{
		uint32_t id = 0;
		std::string name;
		bool alongV = true; // true: isoDirection == "v" (fixed u, vary v); false: isoDirection == "u" (fixed v, vary u)
	};

	struct FinishingIntersectionSpec
	{
		uint32_t idA = 0;
		std::string nameA;
		uint32_t idB = 0;
		std::string nameB;
		std::vector<float3> cursorSeeds;
	};

	struct FinishingConfig
	{
		std::vector<FinishingSurfaceSpec> surfaces;
		std::vector<FinishingIntersectionSpec> intersections;
	};

	struct UVSegmentWithOther
	{
		float u0, v0, u1, v1;
		float ou0, ov0, ou1, ov1;
	};

	struct SurfaceCurveAttachment
	{
		SceneObject* otherObj = nullptr; // nullptr for the offset horizontal base plane
		std::shared_ptr<const GeometryMath::IParametricSurface> otherSurf;
		std::vector<UVSegmentWithOther> segs;
		std::vector<Vec2f> uvSelf;
		std::vector<Vec2f> uvOther;
		std::vector<Vec3f> pointsWorld;
		bool isClosedLoop = false;
	};

	struct TracedFinishingIntersection
	{
		SceneObject* objA = nullptr;
		SceneObject* objB = nullptr;
		std::vector<Vec3f> pointsWorld;
		std::vector<Vec4f> params;
		bool isClosedLoop = false;
	};

	using OffsetSurfaceMap = std::unordered_map<SceneObject*, std::shared_ptr<GeometryMath::OffsetSurface>>;

	FinishingConfig LoadFinishingConfigFromJson(const std::string& filepath)
	{
		FinishingConfig cfg;
		if (filepath.empty())
			return cfg;

		std::ifstream in(filepath);
		if (!in.is_open())
			return cfg;

		try
		{
			nlohmann::json j;
			in >> j;

			if (j.contains("surfaces") && j["surfaces"].is_array())
			{
				for (const auto& item : j["surfaces"])
				{
					FinishingSurfaceSpec spec;
					if (item.contains("id") && item["id"].is_number_integer())
						spec.id = item["id"].get<uint32_t>();
					if (item.contains("name") && item["name"].is_string())
						spec.name = item["name"].get<std::string>();
					if (item.contains("isoDirection") && item["isoDirection"].is_string())
					{
						std::string dir = item["isoDirection"].get<std::string>();
						if (!dir.empty() && (dir[0] == 'u' || dir[0] == 'U'))
							spec.alongV = false;
					}
					if (spec.id != 0 || !spec.name.empty())
						cfg.surfaces.push_back(std::move(spec));
				}
			}

			if (j.contains("intersections") && j["intersections"].is_array())
			{
				for (const auto& item : j["intersections"])
				{
					FinishingIntersectionSpec ispec;
					if (item.contains("idA") && item["idA"].is_number_integer())
						ispec.idA = item["idA"].get<uint32_t>();
					if (item.contains("nameA") && item["nameA"].is_string())
						ispec.nameA = item["nameA"].get<std::string>();
					if (item.contains("idB") && item["idB"].is_number_integer())
						ispec.idB = item["idB"].get<uint32_t>();
					if (item.contains("nameB") && item["nameB"].is_string())
						ispec.nameB = item["nameB"].get<std::string>();
					if (item.contains("cursorSeeds") && item["cursorSeeds"].is_array())
						ispec.cursorSeeds = item["cursorSeeds"].get<std::vector<float3>>();

					if ((ispec.idA != 0 || !ispec.nameA.empty()) &&
						(ispec.idB != 0 || !ispec.nameB.empty()))
					{
						cfg.intersections.push_back(std::move(ispec));
					}
				}
			}
		}
		catch (...)
		{
			return {};
		}

		return cfg;
	}

	std::vector<UVSegmentWithOther> BuildUnwrappedUVSegments(
		const GeometryMath::IParametricSurface& surf,
		const GeometryMath::IParametricSurface& otherSurf,
		const std::vector<Vec4f>& params,
		bool isSurfaceA)
	{
		std::vector<UVSegmentWithOther> segs;
		if (params.size() < 2)
			return segs;

		const float maxU = surf.GetMaxU();
		const float maxV = surf.GetMaxV();
		const float threshU = maxU * 0.5f;
		const float threshV = maxV * 0.5f;
		const float maxOU = otherSurf.GetMaxU();
		const float maxOV = otherSurf.GetMaxV();
		const float threshOU = maxOU * 0.5f;
		const float threshOV = maxOV * 0.5f;

		segs.reserve(params.size());
		for (size_t i = 0; i + 1 < params.size(); i++)
		{
			float u0 = isSurfaceA ? params[i].x : params[i].z;
			float v0 = isSurfaceA ? params[i].y : params[i].w;
			float u1 = isSurfaceA ? params[i + 1].x : params[i + 1].z;
			float v1 = isSurfaceA ? params[i + 1].y : params[i + 1].w;

			float ou0 = isSurfaceA ? params[i].z : params[i].x;
			float ov0 = isSurfaceA ? params[i].w : params[i].y;
			float ou1 = isSurfaceA ? params[i + 1].z : params[i + 1].x;
			float ov1 = isSurfaceA ? params[i + 1].w : params[i + 1].y;

			float dou = ou1 - ou0;
			float dov = ov1 - ov0;
			if (std::abs(dou) > threshOU)
				ou1 += (dou > 0.0f) ? -maxOU : maxOU;
			if (std::abs(dov) > threshOV)
				ov1 += (dov > 0.0f) ? -maxOV : maxOV;

			float du = u1 - u0;
			float dv = v1 - v0;

			if (std::abs(du) > threshU)
			{
				float u1Shift = (du > 0.0f) ? (u1 - maxU) : (u1 + maxU);
				float u0Shift = (du > 0.0f) ? (u0 + maxU) : (u0 - maxU);
				segs.push_back({ u0, v0, u1Shift, v1, ou0, ov0, ou1, ov1 });
				segs.push_back({ u0Shift, v0, u1, v1, ou0, ov0, ou1, ov1 });
			}
			else if (std::abs(dv) > threshV)
			{
				float v1Shift = (dv > 0.0f) ? (v1 - maxV) : (v1 + maxV);
				float v0Shift = (dv > 0.0f) ? (v0 + maxV) : (v0 - maxV);
				segs.push_back({ u0, v0, u1, v1Shift, ou0, ov0, ou1, ov1 });
				segs.push_back({ u0, v0Shift, u1, v1, ou0, ov0, ou1, ov1 });
			}
			else
			{
				segs.push_back({ u0, v0, u1, v1, ou0, ov0, ou1, ov1 });
			}
		}

		return segs;
	}

	struct FinishingIsoPassSegment
	{
		float alpha = 0.0f;
		float betaStart = 0.0f;
		float betaEnd = 0.0f;
		std::vector<float3> pts; // cutter tip waypoints in scene mm
	};

	float WrapOrClampParam(float val, float maxVal, bool isWrapped)
	{
		if (isWrapped && maxVal > 0.0f)
		{
			val = std::fmod(val, maxVal);
			if (val < 0.0f)
				val += maxVal;
			return val;
		}
		return std::clamp(val, 0.0f, maxVal);
	}

	std::vector<float3> SimplifyPolyline3DGougeSafe(const std::vector<float3>& rawPts, float tolerance)
	{
		if (rawPts.size() <= 2 || tolerance <= 0.0f)
			return rawPts;

		const int n = static_cast<int>(rawPts.size());
		std::vector<bool> keep(n, false);
		keep.front() = true;
		keep.back() = true;

		const float tolSq = tolerance * tolerance;
		constexpr float cUndercutEps = 0.004f; // Never allow chord to dip > 4 microns below intermediate points

		std::vector<std::pair<int, int>> stack;
		stack.reserve(n);
		stack.push_back({ 0, n - 1 });

		while (!stack.empty())
		{
			auto [iStart, iEnd] = stack.back();
			stack.pop_back();

			if (iEnd - iStart <= 1)
				continue;

			const Vec3f A = rawPts[iStart].ToVec3f();
			const Vec3f B = rawPts[iEnd].ToVec3f();
			const Vec3f AB = B - A;
			const float abLenSq = AB.length_sqr();

			float maxDistSq = 0.0f;
			int maxIdx = -1;
			float worstUndercut = 0.0f;
			int worstUndercutIdx = -1;

			for (int i = iStart + 1; i < iEnd; i++)
			{
				const Vec3f P = rawPts[i].ToVec3f();
				float s = (abLenSq > 1e-10f)
					? std::clamp(Vec3f::dot(P - A, AB) / abLenSq, 0.0f, 1.0f)
					: (static_cast<float>(i - iStart) / (iEnd - iStart));

				const Vec3f proj = A + AB * s;
				const float distSq = (P - proj).length_sqr();
				if (distSq > maxDistSq)
				{
					maxDistSq = distSq;
					maxIdx = i;
				}

				const float yDiff = proj.y - P.y;
				if (yDiff < -cUndercutEps && yDiff < worstUndercut)
				{
					worstUndercut = yDiff;
					worstUndercutIdx = i;
				}
			}

			int splitIdx = -1;
			if (worstUndercutIdx != -1)
				splitIdx = worstUndercutIdx;
			else if (maxDistSq > tolSq)
				splitIdx = maxIdx;

			if (splitIdx != -1)
			{
				keep[splitIdx] = true;
				stack.push_back({ iStart, splitIdx });
				stack.push_back({ splitIdx, iEnd });
			}
		}

		std::vector<float3> result;
		result.reserve(n);
		for (int i = 0; i < n; i++)
		{
			if (keep[i])
				result.push_back(rawPts[i]);
		}
		return result;
	}

	std::vector<std::pair<float, float>> ComputeValidFlowIntervals(
		const GeometryMath::OffsetSurface& clSelf,
		bool alongV,
		float alphaScan,
		const std::vector<SurfaceCurveAttachment>& attachments,
		float yCenterMinWorld)
	{
		const float maxBeta = alongV ? clSelf.GetMaxV() : clSelf.GetMaxU();
		auto evalAB = [&](float a, float b) -> GeometryMath::SurfaceEvalResult
			{
				return alongV ? clSelf.Evaluate(a, b) : clSelf.Evaluate(b, a);
			};

		std::vector<std::pair<float, float>> intervals = { { 0.0f, maxBeta } };

		struct FlowHit
		{
			float bHit = 0.0f;
			int transSign = 1;
		};

		for (const auto& att : attachments)
		{
			std::vector<FlowHit> hits;
			for (const auto& seg : att.segs)
			{
				float a0 = alongV ? seg.u0 : seg.v0;
				float b0 = alongV ? seg.v0 : seg.u0;
				float a1 = alongV ? seg.u1 : seg.v1;
				float b1 = alongV ? seg.v1 : seg.u1;

				if ((a0 <= alphaScan && alphaScan < a1) || (a1 <= alphaScan && alphaScan < a0))
				{
					float da = a1 - a0;
					if (std::abs(da) < 1e-9f)
						continue;

					float t = (alphaScan - a0) / da;
					float bHit = b0 + t * (b1 - b0);
					float du = seg.u1 - seg.u0;
					float dv = seg.v1 - seg.v0;
					float ou = seg.ou0 + t * (seg.ou1 - seg.ou0);
					float ov = seg.ov0 + t * (seg.ov1 - seg.ov0);

					float uHit = alongV ? alphaScan : bHit;
					float vHit = alongV ? bHit : alphaScan;
					auto evalA = clSelf.Evaluate(uHit, vHit);
					auto evalB = att.otherSurf->Evaluate(ou, ov);
					Vec3f nB = Vec3f::cross(evalB.du, evalB.dv);
					float nBLenSq = nB.length_sqr();
					if (nBLenSq <= 1e-12f)
						continue;
					nB /= std::sqrt(nBLenSq);

					Vec3f dPLeft = evalA.dv * du - evalA.du * dv;
					float leftDot = Vec3f::dot(nB, dPLeft);
					float crossScan = alongV ? du : (-dv);
					float trans = crossScan * leftDot;
					hits.push_back({ bHit, (trans > 0.0f) ? 1 : -1 });
				}
			}

			std::vector<std::pair<float, float>> forbidden;
			if (hits.empty())
			{
				if (att.otherObj != nullptr)
				{
					int bestIdx = -1;
					float bestDa = std::numeric_limits<float>::max();
					float minBAbove = std::numeric_limits<float>::max();
					float maxBAbove = -std::numeric_limits<float>::max();

					for (size_t i = 0; i < att.pointsWorld.size(); i++)
					{
						if (att.pointsWorld[i].y >= yCenterMinWorld - 1e-4f)
						{
							float aVal = alongV ? att.uvSelf[i].x : att.uvSelf[i].y;
							float bVal = alongV ? att.uvSelf[i].y : att.uvSelf[i].x;
							float d = std::abs(aVal - alphaScan);
							if (d < bestDa)
							{
								bestDa = d;
								bestIdx = static_cast<int>(i);
							}
							minBAbove = std::min(minBAbove, bVal);
							maxBAbove = std::max(maxBAbove, bVal);
						}
					}

					if (bestIdx != -1)
					{
						float uStar = att.uvSelf[bestIdx].x;
						float vStar = att.uvSelf[bestIdx].y;
						float aStar = alongV ? uStar : vStar;
						float ouStar = att.uvOther[bestIdx].x;
						float ovStar = att.uvOther[bestIdx].y;

						auto evalA = clSelf.Evaluate(uStar, vStar);
						auto evalB = att.otherSurf->Evaluate(ouStar, ovStar);
						Vec3f nB = Vec3f::cross(evalB.du, evalB.dv);
						Vec3f dAlphaA = alongV ? evalA.du : evalA.dv;
						float nBLenSq = nB.length_sqr();
						float dAlphaLen = dAlphaA.length();
						if (nBLenSq > 1e-12f && dAlphaLen > 1e-6f)
						{
							nB /= std::sqrt(nBLenSq);
							float dotA = Vec3f::dot(nB, dAlphaA) / dAlphaLen;
							if (std::abs(dotA) > 0.3f && (alphaScan - aStar) * dotA < 0.0f)
							{
								float padB = 0.05f * maxBeta;
								forbidden.push_back({ minBAbove - padB, maxBAbove + padB });
							}
						}
					}
				}
			}
			else
			{
				std::sort(hits.begin(), hits.end(), [](const FlowHit& a, const FlowHit& b) {
					return a.bHit < b.bHit;
					});

				std::vector<FlowHit> dedup;
				dedup.reserve(hits.size());
				dedup.push_back(hits.front());
				for (size_t i = 1; i < hits.size(); i++)
				{
					if (std::abs(hits[i].bHit - dedup.back().bHit) > 1e-3f)
						dedup.push_back(hits[i]);
				}

				if (dedup.front().transSign == +1)
					forbidden.push_back({ -1.0f, dedup.front().bHit });
				for (size_t k = 0; k + 1 < dedup.size(); k++)
				{
					if (dedup[k].transSign == -1 || dedup[k + 1].transSign == +1)
						forbidden.push_back({ dedup[k].bHit, dedup[k + 1].bHit });
				}
				if (dedup.back().transSign == -1)
					forbidden.push_back({ dedup.back().bHit, maxBeta + 1.0f });
			}

			if (!forbidden.empty())
			{
				std::vector<std::pair<float, float>> nextIntervals;
				for (const auto& [a, b] : intervals)
				{
					std::vector<std::pair<float, float>> pieces = { { a, b } };
					for (const auto& [fa, fb] : forbidden)
					{
						std::vector<std::pair<float, float>> nextPieces;
						for (const auto& [pa, pb] : pieces)
						{
							if (fb <= pa || fa >= pb)
							{
								nextPieces.push_back({ pa, pb });
							}
							else
							{
								if (pa < fa - 1e-5f)
									nextPieces.push_back({ pa, fa });
								if (fb + 1e-5f < pb)
									nextPieces.push_back({ fb, pb });
							}
						}
						pieces = std::move(nextPieces);
					}
					nextIntervals.insert(nextIntervals.end(), pieces.begin(), pieces.end());
				}
				intervals = std::move(nextIntervals);
				if (intervals.empty())
					return intervals;
			}
		}

		std::vector<std::pair<float, float>> validIntervals;
		validIntervals.reserve(intervals.size());
		for (const auto& [a, b] : intervals)
		{
			if (b <= a + 1e-5f)
				continue;
			float bMid = 0.5f * (a + b);
			if (evalAB(alphaScan, bMid).p.y >= yCenterMinWorld - 1e-3f)
				validIntervals.push_back({ a, b });
		}

		return validIntervals;
	}

	void CollectBasePlaneIntersections(
		SceneObject* obj,
		const GeometryMath::OffsetSurface& clSelf,
		const std::shared_ptr<GeometryMath::HorizontalPlaneSurface>& horizPlane,
		float stepWorld,
		const std::vector<std::pair<uint32_t, std::vector<float3>>>& baseSeedsById,
		std::vector<SurfaceCurveAttachment>& outAttachments)
	{
		const Vec4f maxDomains(clSelf.GetMaxU(), clSelf.GetMaxV(), horizPlane->GetMaxU(), horizPlane->GetMaxV());

		auto traceWithSeed = [&](std::optional<float3> seed)
			{
				const bool allowOpenDirect = seed.has_value();
				auto resOpt = GeometryMath::CalculateIntersection(clSelf, *horizPlane, false, maxDomains, stepWorld, seed, allowOpenDirect);
				if (!resOpt && !allowOpenDirect)
					resOpt = GeometryMath::CalculateIntersection(clSelf, *horizPlane, false, maxDomains, stepWorld, seed, true);

				if (!resOpt || resOpt->points.size() < 2)
					return;

				std::vector<Vec2f> uvSelf(resOpt->params.size());
				std::vector<Vec2f> uvOther(resOpt->params.size());
				for (size_t k = 0; k < resOpt->params.size(); k++)
				{
					uvSelf[k] = { resOpt->params[k].x, resOpt->params[k].y };
					uvOther[k] = { resOpt->params[k].z, resOpt->params[k].w };
				}

				outAttachments.push_back({
					nullptr,
					horizPlane,
					BuildUnwrappedUVSegments(clSelf, *horizPlane, resOpt->params, true),
					std::move(uvSelf),
					std::move(uvOther),
					std::move(resOpt->points),
					resOpt->isClosedLoop
					});
			};

		auto seedIt = std::find_if(baseSeedsById.begin(), baseSeedsById.end(),
			[id = obj->m_id](const auto& entry) { return entry.first == id; });
		if (seedIt != baseSeedsById.end() && !seedIt->second.empty())
		{
			for (const auto& seed : seedIt->second)
				traceWithSeed(seed);
		}
		else
		{
			traceWithSeed(std::nullopt);
		}
	}
}

std::vector<float3> PathGenerator::GenerateFlatBasePath(
	const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
	const FlatBaseParameters& params)
{
	const float halfX = params.blockSizeX * 0.5f;
	const float halfZ = params.blockSizeZ * 0.5f;
	const float R = params.cutterDiameter * 0.5f;
	const float margin = std::max(0.1f, params.contourMargin);
	const float stepover = std::clamp(params.stepover, 0.5f, params.cutterDiameter * 0.95f);
	const float safeY = std::max(params.safeHeight, params.blockSizeY + 10.0f);
	const float outsideLeftX = -halfX - R - 2.0f;
	const float outsideRightX = halfX + R + 2.0f;
	constexpr float cSimplifyTol = 0.03f;

	float baseY = params.baseHeight;
	if (params.baseSurface != nullptr)
	{
		auto eval = GeometryMath::EvaluateParametric(params.baseSurface, 0.5f, 0.5f);
		baseY = std::max(baseY, eval.p.y * cWorldToMm);
	}
	const float yBaseWorld = baseY * cMmToWorld;

	// Oversized virtual horizontal plane so CL intersections close cleanly outside block edges
	const float planePadWorld = 4.0f; // 80 mm extra per side
	GeometryMath::HorizontalPlaneSurface horizPlane(
		yBaseWorld,
		-halfX * cMmToWorld - planePadWorld,
		halfX * cMmToWorld + planePadWorld,
		-halfZ * cMmToWorld - planePadWorld,
		halfZ * cMmToWorld + planePadWorld);

	// 1. Compute exact CL outer contour (offset R) and zigzag clearance contour (offset R + margin)
	auto seedEntries = LoadBaseCursorSeedsFromJson(params.seedsJsonPath);
	auto outerContours = CollectHorizontalCLContours(sceneObjects, params.baseSurface, yBaseWorld, R, horizPlane, seedEntries);
	auto zigzagContours = (margin > 1e-4f)
		? CollectHorizontalCLContours(sceneObjects, params.baseSurface, yBaseWorld, R + margin, horizPlane, seedEntries)
		: outerContours;

	std::vector<Vec2f> clContour = outerContours.empty() ? std::vector<Vec2f>{} : outerContours.front();
	std::vector<Vec2f> zigContour = zigzagContours.empty() ? clContour : zigzagContours.front();

	auto rotateToMinZ = [](std::vector<Vec2f>& contour)
		{
			if (!contour.empty())
			{
				auto minIt = std::min_element(contour.begin(), contour.end(),
					[](const Vec2f& a, const Vec2f& b) { return a.z < b.z; });
				std::rotate(contour.begin(), minIt, contour.end());
			}
		};
	rotateToMinZ(clContour);
	rotateToMinZ(zigContour);

	std::vector<float3> waypoints;
	waypoints.reserve(1024);
	if (zigContour.size() < 3)
		return waypoints;

	BaseWaypointWriter writer{ waypoints, baseY };

	// 2. Plan uniform Z scanlines across the block respecting the exact requested stepover
	const int numPasses = std::max(1, static_cast<int>(std::ceil(params.blockSizeZ / stepover)));
	std::vector<float> zPasses(numPasses + 1);
	for (int p = 0; p <= numPasses; p++)
		zPasses[p] = -halfZ + (params.blockSizeZ * p) / numPasses;

	const float zMinObj = zigContour.front().z;
	std::vector<float> topZ, midZ, botZ;
	for (float zp : zPasses)
	{
		if (!IntersectHorizontalWithPolygon2D(zp, zigContour).empty())
			midZ.push_back(zp);
		else if (zp < zMinObj)
			topZ.push_back(zp);
		else
			botZ.push_back(zp);
	}

	// 3. Continuous non-backtracking zigzag at constant height baseY (only first and last points at safeY)
	writer.Append(outsideLeftX, safeY, zPasses.front());

	// 3a. Top full-width passes (starts on the left: currRight == false)
	bool currRight = false;
	for (float zp : topZ)
	{
		writer.AppendBase(currRight ? outsideRightX : outsideLeftX, zp);
		writer.AppendBase(currRight ? outsideLeftX : outsideRightX, zp);
		currRight = !currRight;
	}

	const auto midOpenBays = midZ.empty()
		? std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>>{}
		: CollectOpenBaysAtZ(zigContour, midZ.back());

	// If midZ has an odd count and midZ[0] cleanly spans the top tip, bridge across midZ[0] to pair the remaining mid lines
	if (midZ.size() % 2 == 1)
	{
		auto hits0 = IntersectHorizontalWithPolygon2D(midZ.front(), zigContour);
		if (hits0.size() >= 2)
		{
			float z0 = midZ.front();
			midZ.erase(midZ.begin());

			const HorizontalHit2D& hStart = currRight ? hits0.back() : hits0.front();
			const HorizontalHit2D& hEnd = currRight ? hits0.front() : hits0.back();

			writer.AppendBase(currRight ? outsideRightX : outsideLeftX, z0);
			writer.AppendBase(hStart.pt.x, hStart.pt.z);

			std::vector<Vec2f> simpArc = SimplifyPolyline2D(ExtractShorterArc2D(zigContour, hStart, hEnd), cSimplifyTol);
			for (size_t m = 1; m < simpArc.size(); m++)
				writer.AppendBase(simpArc[m].x, simpArc[m].z);

			writer.AppendBase(currRight ? outsideLeftX : outsideRightX, z0);
			currRight = !currRight;
		}
	}

	// 3b. First descending side along midZ
	TraceSideZigzagPasses(writer, zigContour, midZ, currRight, currRight ? outsideRightX : outsideLeftX, cSimplifyTol);

	// 3c. Bottom passes crossing from currRight to !currRight (including open bay loop under the waves)
	AppendBottomPassesAndBays(writer, botZ, midOpenBays, currRight, outsideLeftX, outsideRightX, halfZ, R);

	// 3d. Opposite ascending side going bottom-to-top
	std::reverse(midZ.begin(), midZ.end());
	TraceSideZigzagPasses(writer, zigContour, midZ, !currRight, (!currRight) ? outsideRightX : outsideLeftX, cSimplifyTol);

	// 4. Enter clContour at its global min-Z apex, trace full closed contour, then exit in -Z outside the block
	AppendContourPass(writer, clContour, halfZ, R, cSimplifyTol);

	// 5. Final retract in air outside the block to safe height
	const float3& lastPt = waypoints.back();
	writer.Append(lastPt.x, safeY, lastPt.z);

	return waypoints;
}

std::vector<float3> PathGenerator::GenerateRoughingPath(
	const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
	const RoughingParameters& params)
{
	const int divX = std::max(2, params.gridDivX);
	const int divZ = std::max(2, params.gridDivZ);
	const int cols = divX + 1;
	const float halfX = params.blockSizeX * 0.5f;
	const float halfZ = params.blockSizeZ * 0.5f;
	const float dx = params.blockSizeX / divX;
	const float dz = params.blockSizeZ / divZ;
	const float R = params.cutterDiameter * 0.5f;

	RoughingParameters effectiveParams = params;
	effectiveParams.gridDivX = divX;
	effectiveParams.gridDivZ = divZ;

	if (effectiveParams.baseSurface != nullptr)
	{
		auto eval = GeometryMath::EvaluateParametric(effectiveParams.baseSurface, 0.5f, 0.5f);
		effectiveParams.baseHeight = std::max(effectiveParams.baseHeight, eval.p.y * cWorldToMm);
	}

	// 1. Build ball-nose cutter tip height map directly from offset surfaces (P + R_eff * n) and C0 Bezier normal cones
	std::vector<float> clGrid = BuildCutterLocationHeightMap(sceneObjects, effectiveParams);

	// 3. Determine two-layer minimum heights respecting flute height (2 * diameter = 32 mm) and base height (> 15 mm)
	const float minLayer2Y = effectiveParams.baseHeight + std::max(0.1f, effectiveParams.baseMargin);
	const float minLayer1Y = std::clamp(
		effectiveParams.layer1Height,
		effectiveParams.blockSizeY - effectiveParams.cutterFluteHeight + 1.0f,
		effectiveParams.blockSizeY - 1.0f);

	const float layerMinHeights[2] = { minLayer1Y, minLayer2Y };

	// Determine Z scanline indices covering [-halfZ, +halfZ] with step <= stepover
	const float stepover = std::clamp(effectiveParams.stepover, 0.5f, effectiveParams.cutterDiameter * 0.9f);
	const int numPassIntervals = std::max(1, static_cast<int>(std::ceil(effectiveParams.blockSizeZ / stepover)));

	std::vector<int> passIzIndices;
	passIzIndices.reserve(numPassIntervals + 1);
	for (int p = 0; p <= numPassIntervals; p++)
	{
		int iz = std::clamp(static_cast<int>(std::round(static_cast<float>(p) * divZ / numPassIntervals)), 0, divZ);
		if (passIzIndices.empty() || passIzIndices.back() != iz)
			passIzIndices.push_back(iz);
	}

	auto getLayerY = [&](int ix, int iz, float layerMinY) -> float
		{
			return std::max(clGrid[static_cast<size_t>(iz) * cols + ix], layerMinY);
		};

	std::vector<float3> waypoints;
	waypoints.reserve(passIzIndices.size() * 64);

	const float safeY = std::max(effectiveParams.safeHeight, effectiveParams.blockSizeY + 10.0f);
	const float outsideLeftX = -halfX - R - 2.0f;
	const float outsideRightX = halfX + R + 2.0f;
	const size_t numPasses = passIzIndices.size();

	// 1. Initial positioning in air above entry point outside the block
	const float firstZ = -halfZ + passIzIndices.front() * dz;
	waypoints.emplace_back(outsideLeftX, safeY, firstZ);

	// 2. Multi-layer bidirectional roughing passes (Layer 1 forward in Z, Layer 2 backward in Z)
	bool leftToRight = true;

	for (int layer = 0; layer < 2; layer++)
	{
		const float layerMinY = layerMinHeights[layer];
		const bool forwardZ = (layer == 0);

		for (size_t p = 0; p < numPasses; p++)
		{
			const int iz = forwardZ ? passIzIndices[p] : passIzIndices[numPasses - 1 - p];
			const float pz = -halfZ + iz * dz;

			std::vector<float3> rawScanline;
			rawScanline.reserve(cols + 2);

			if (leftToRight)
			{
				rawScanline.emplace_back(outsideLeftX, getLayerY(0, iz, layerMinY), pz);
				for (int ix = 0; ix <= divX; ix++)
					rawScanline.emplace_back(-halfX + ix * dx, getLayerY(ix, iz, layerMinY), pz);
				rawScanline.emplace_back(outsideRightX, getLayerY(divX, iz, layerMinY), pz);
			}
			else
			{
				rawScanline.emplace_back(outsideRightX, getLayerY(divX, iz, layerMinY), pz);
				for (int ix = divX; ix >= 0; ix--)
					rawScanline.emplace_back(-halfX + ix * dx, getLayerY(ix, iz, layerMinY), pz);
				rawScanline.emplace_back(outsideLeftX, getLayerY(0, iz, layerMinY), pz);
			}

			std::vector<float3> simplifiedScanline = SimplifyScanlineSafe(rawScanline, effectiveParams.simplifyTolerance);
			for (const auto& pt : simplifiedScanline)
			{
				if (waypoints.empty() || (pt.ToVec3f() - waypoints.back().ToVec3f()).length_sqr() > 1e-6f)
					waypoints.push_back(pt);
			}

			leftToRight = !leftToRight;
		}
	}

	// Final retract in air outside the block to safe height
	const float3& lastPt = waypoints.back();
	waypoints.emplace_back(lastPt.x, safeY, lastPt.z);

	return waypoints;
}

namespace
{
	struct FinishingClearanceMap
	{
		int divX = 250;
		int divZ = 250;
		float halfX = 75.0f;
		float halfZ = 75.0f;
		float invDx = 1.0f;
		float invDz = 1.0f;
		float minSafeTipY = 15.0f;
		float maxModelSurfY = 15.0f;
		std::vector<float> grid;

		float SampleMax(float xMm, float zMm) const
		{
			const float gx = std::clamp((xMm + halfX) * invDx, 0.0f, static_cast<float>(divX));
			const float gz = std::clamp((zMm + halfZ) * invDz, 0.0f, static_cast<float>(divZ));
			const int ix0 = std::min(static_cast<int>(std::floor(gx)), divX);
			const int iz0 = std::min(static_cast<int>(std::floor(gz)), divZ);
			const int ix1 = std::min(ix0 + 1, divX);
			const int iz1 = std::min(iz0 + 1, divZ);
			const int cols = divX + 1;
			const float h00 = grid[static_cast<size_t>(iz0) * cols + ix0];
			const float h10 = grid[static_cast<size_t>(iz0) * cols + ix1];
			const float h01 = grid[static_cast<size_t>(iz1) * cols + ix0];
			const float h11 = grid[static_cast<size_t>(iz1) * cols + ix1];
			return std::max({ minSafeTipY, h00, h10, h01, h11 });
		}

		float MaxAlongCorridor(const Vec3f& a, const Vec3f& b) const
		{
			const float distXZ = std::hypot(b.x - a.x, b.z - a.z);
			const int steps = std::clamp(static_cast<int>(std::ceil(distXZ / 0.6f)), 2, 200);
			float maxH = minSafeTipY;
			for (int s = 0; s <= steps; s++)
			{
				const float t = static_cast<float>(s) / steps;
				maxH = std::max(maxH, SampleMax(std::lerp(a.x, b.x, t), std::lerp(a.z, b.z, t)));
			}
			return maxH;
		}
	};

	FinishingClearanceMap BuildFinishingClearanceMap(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const FinishingParameters& params,
		float cutterRadiusMm,
		float baseY,
		float minSafeTipY)
	{
		FinishingClearanceMap map{};
		map.divX = 250;
		map.divZ = 250;
		map.halfX = params.blockSizeX * 0.5f;
		map.halfZ = params.blockSizeZ * 0.5f;
		map.invDx = static_cast<float>(map.divX) / params.blockSizeX;
		map.invDz = static_cast<float>(map.divZ) / params.blockSizeZ;
		map.minSafeTipY = minSafeTipY;

		RoughingParameters rp{};
		rp.blockSizeX = params.blockSizeX;
		rp.blockSizeZ = params.blockSizeZ;
		rp.baseHeight = baseY;
		rp.cutterDiameter = cutterRadiusMm * 2.0f;
		rp.surfaceMargin = 0.0f;
		rp.baseMargin = 0.0f;
		rp.gridDivX = map.divX;
		rp.gridDivZ = map.divZ;
		rp.baseSurface = params.baseSurface;

		map.grid = BuildCutterLocationHeightMap(sceneObjects, rp);
		float maxTipY = baseY;
		for (float tipY : map.grid)
			maxTipY = std::max(maxTipY, tipY);
		map.maxModelSurfY = maxTipY;
		return map;
	}

	struct FinishingPathWriter
	{
		static constexpr float cMinSafeHorizontalSlope = 0.048f; // ~2.75 deg from vertical (> 1.5 deg threshold)

		std::vector<float3> waypoints;
		int totalLifts = 0;
		double totalAirMoveMm = 0.0;

		float halfX = 75.0f;
		float halfZ = 75.0f;
		float R = 4.0f;
		float baseY = 15.0f;
		float minSafeTipY = 15.015f;
		float safeY = 60.0f;
		float globalModelClearY = 50.0f;
		const FinishingClearanceMap& clearance;

		struct ExteriorAccessInfo
		{
			float outX = 0.0f;
			float outZ = 0.0f;
			float corridorIto = 0.0f;
			float score = std::numeric_limits<float>::max();
		};

		double MeasureLenFrom(size_t startIdx) const
		{
			if (waypoints.size() <= startIdx + 1)
				return 0.0;
			double sum = 0.0;
			for (size_t i = startIdx + 1; i < waypoints.size(); i++)
				sum += (waypoints[i].ToVec3f() - waypoints[i - 1].ToVec3f()).length();
			return sum;
		}

		void AppendSafePoint(const float3& targetPt)
		{
			float3 pt = targetPt;
			pt.y = std::max(pt.y, minSafeTipY);
			if (waypoints.empty())
			{
				waypoints.push_back(pt);
				return;
			}
			const Vec3f prev = waypoints.back().ToVec3f();
			const float dxMove = pt.x - prev.x;
			const float dyMove = pt.y - prev.y;
			const float dzMove = pt.z - prev.z;
			if ((pt.ToVec3f() - prev).length_sqr() < 1e-6f)
				return;
			if (dyMove < -1e-4f)
			{
				const float lenXZ = std::sqrt(dxMove * dxMove + dzMove * dzMove);
				const float minRequiredXZ = (-dyMove) * cMinSafeHorizontalSlope;
				if (lenXZ < minRequiredXZ)
				{
					const float dirX = (lenXZ > 1e-5f) ? (dxMove / lenXZ) : 1.0f;
					const float dirZ = (lenXZ > 1e-5f) ? (dzMove / lenXZ) : 0.0f;
					pt.x = prev.x + dirX * minRequiredXZ;
					pt.z = prev.z + dirZ * minRequiredXZ;
				}
			}
			waypoints.push_back(pt);
		}

		ExteriorAccessInfo EvaluateExteriorAccess(const Vec3f& dest) const
		{
			const float padEdge = R + 2.5f;
			const std::pair<float, float> candidates[4] = {
				{ +halfX + padEdge, dest.z },
				{ -halfX - padEdge, dest.z },
				{ dest.x, +halfZ + padEdge },
				{ dest.x, -halfZ - padEdge }
			};

			ExteriorAccessInfo best{};
			for (int c = 0; c < 4; c++)
			{
				const float cx = candidates[c].first;
				const float cz = candidates[c].second;
				const float dist = std::hypot(cx - dest.x, cz - dest.z);
				const float corridorMax = clearance.MaxAlongCorridor({ cx, dest.y, cz }, dest);
				const float score = corridorMax * 100.0f + dist;
				if (score < best.score)
				{
					best.score = score;
					best.outX = cx;
					best.outZ = cz;
					best.corridorIto = corridorMax;
				}
			}
			return best;
		}

		// aheadPts: the next vertices of the path that starts at targetPt (nearest first, ~3 mm of path).
		// The descent onto targetPt is then performed along those vertices (same XZ footprint as the path itself),
		// so the cutter never drops diagonally from the other side of a (vertical) wall that the path hugs.
		void TransitionSafeCl(const float3& targetPt, const std::vector<float3>* aheadPts = nullptr)
		{
			float3 dest = targetPt;
			dest.y = std::max(dest.y, minSafeTipY);

			if (waypoints.empty())
			{
				const ExteriorAccessInfo access = EvaluateExteriorAccess(dest.ToVec3f());
				const float entryY = std::min(globalModelClearY, std::max({ dest.y + 2.5f, access.corridorIto + 2.5f }));

				waypoints.emplace_back(access.outX, safeY, access.outZ);
				if (entryY < safeY - 1e-3f)
					waypoints.emplace_back(access.outX, entryY, access.outZ);

				const float dropY = std::max(0.0f, entryY - dest.y);
				const float rampDist = std::clamp(dropY * 1.15f, 3.0f, 12.0f);
				const float toOutX = access.outX - dest.x;
				const float toOutZ = access.outZ - dest.z;
				const float toOutLen = std::hypot(toOutX, toOutZ);
				if (toOutLen > rampDist + 2.0f)
				{
					AppendSafePoint({ dest.x + (toOutX / toOutLen) * rampDist, entryY, dest.z + (toOutZ / toOutLen) * rampDist });
				}

				AppendSafePoint(dest);
				totalLifts++;
				totalAirMoveMm += MeasureLenFrom(0);
				return;
			}

			const size_t wpBeforeTransfer = waypoints.size() - 1;
			const float3 last = waypoints.back();
			const float toLastX = last.x - dest.x;
			const float toLastZ = last.z - dest.z;
			const float distXZ = std::hypot(toLastX, toLastZ);

			const float maxCorridorItoY = clearance.MaxAlongCorridor(last.ToVec3f(), dest.ToVec3f());
			const float travelY = std::max({
				last.y,
				dest.y,
				std::min(globalModelClearY, std::max({ last.y + 0.8f, dest.y + 0.8f, maxCorridorItoY + 2.2f }))
				});

			if (last.y < travelY - 1e-3f)
				AppendSafePoint({ last.x, travelY, last.z });

			if (aheadPts && !aheadPts->empty())
			{
				std::vector<float> cum(aheadPts->size(), 0.0f);
				float maxAheadY = dest.y;
				float prevX = dest.x, prevZ = dest.z, acc = 0.0f;
				for (size_t k = 0; k < aheadPts->size(); k++)
				{
					const float3& ap = (*aheadPts)[k];
					acc += std::hypot(ap.x - prevX, ap.z - prevZ);
					cum[k] = acc;
					prevX = ap.x; prevZ = ap.z;
					maxAheadY = std::max(maxAheadY, ap.y);
				}
				const float3& farPt = aheadPts->back();
				float approachY = std::max({ travelY, maxAheadY + 0.6f, dest.y + 0.8f });
				const float farCorridorY = clearance.MaxAlongCorridor(last.ToVec3f(), farPt.ToVec3f());
				approachY = std::max(approachY, std::min(globalModelClearY, farCorridorY + 2.2f));
				const float approachDrop = approachY - dest.y;
				if (acc >= approachDrop * (cMinSafeHorizontalSlope * 1.5f) + 0.05f)
				{
					if (last.y < approachY - 1e-3f)
						AppendSafePoint({ last.x, approachY, last.z });
					AppendSafePoint({ farPt.x, approachY, farPt.z });
					for (int k = static_cast<int>(aheadPts->size()) - 2; k >= 0; k--)
					{
						const float3& ap = (*aheadPts)[k];
						const float y = std::max(dest.y + approachDrop * (cum[k] / acc), ap.y + 0.25f);
						AppendSafePoint({ ap.x, y, ap.z });
					}
					AppendSafePoint(dest);
					totalLifts++;
					totalAirMoveMm += MeasureLenFrom(wpBeforeTransfer);
					return;
				}
			}

			const float dropY = std::max(0.0f, travelY - dest.y);
			const float rampDist = std::clamp(dropY * 0.06f, 0.08f, 0.9f);
			if (distXZ > rampDist + 0.1f)
			{
				AppendSafePoint({ dest.x + (toLastX / distXZ) * rampDist, travelY, dest.z + (toLastZ / distXZ) * rampDist });
			}

			AppendSafePoint(dest);
			totalLifts++;
			totalAirMoveMm += MeasureLenFrom(wpBeforeTransfer);
		}

	};

	struct PureIntersectionLake
	{
		std::vector<float3> arcA;
		std::vector<float3> arcB;
		float areaMm2 = 0.0f;
	};

	std::vector<PureIntersectionLake> ExtractPureIntersectionLakes(
		const std::vector<std::pair<SceneObject*, bool>>& orderedSurfaces,
		const OffsetSurfaceMap& offsetSurfaces,
		const std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>>& surfAttachments,
		float yCenterMinWorld,
		float halfX,
		float halfZ,
		float minSafeTipY)
	{
		struct TrimmedBaseArc { size_t surfIdx = 0; std::vector<float3> pts; };
		std::vector<TrimmedBaseArc> openBaseArcs;

		for (size_t sIdx = 0; sIdx < orderedSurfaces.size(); sIdx++)
		{
			SceneObject* obj = orderedSurfaces[sIdx].first;
			const bool alongV = orderedSurfaces[sIdx].second;
			const auto& clSelf = *offsetSurfaces.at(obj);
			auto attIt = surfAttachments.find(obj);
			if (attIt == surfAttachments.end())
				continue;

			std::vector<SurfaceCurveAttachment> ssAtts;
			for (const auto& att : attIt->second)
			{
				if (att.otherObj != nullptr)
					ssAtts.push_back(att);
			}

			auto isUVValidAgainstOtherSurfaces = [&](float uVal, float vVal) -> bool
				{
					const float aVal = alongV ? uVal : vVal;
					const float bVal = alongV ? vVal : uVal;
					auto ivs = ComputeValidFlowIntervals(clSelf, alongV, aVal, ssAtts, yCenterMinWorld);
					for (const auto& [b0, b1] : ivs)
					{
						if (bVal >= b0 - 1e-4f && bVal <= b1 + 1e-4f)
							return true;
					}
					return false;
				};

			for (const auto& att : attIt->second)
			{
				if (att.otherObj != nullptr || att.pointsWorld.size() < 2)
					continue;
				std::vector<Vec3f> pts = att.pointsWorld;
				std::vector<Vec2f> uvs = att.uvSelf;
				auto isBasePtValid = [&](size_t k) -> bool
					{
						if (std::abs(pts[k].x * cWorldToMm) > halfX || std::abs(pts[k].z * cWorldToMm) > halfZ)
							return false;
						return isUVValidAgainstOtherSurfaces(uvs[k].x, uvs[k].y);
					};

				int invalidIdx = -1;
				for (size_t k = 0; k < pts.size(); k++)
				{
					if (!isBasePtValid(k)) { invalidIdx = static_cast<int>(k); break; }
				}
				if (invalidIdx < 0)
					continue;

				if (att.isClosedLoop && pts.size() > 3 && invalidIdx > 0)
				{
					pts.pop_back();
					uvs.pop_back();
					std::rotate(pts.begin(), pts.begin() + invalidIdx, pts.end());
					std::rotate(uvs.begin(), uvs.begin() + invalidIdx, uvs.end());
					pts.push_back(pts.front());
					uvs.push_back(uvs.front());
				}

				std::vector<float3> currArc;
				auto flushArc = [&]()
					{
						if (currArc.size() >= 3)
							openBaseArcs.push_back({ sIdx, std::move(currArc) });
						currArc.clear();
					};
				for (size_t k = 0; k < pts.size(); k++)
				{
					if (isBasePtValid(k))
						currArc.emplace_back(pts[k].x * cWorldToMm, minSafeTipY, pts[k].z * cWorldToMm);
					else
						flushArc();
				}
				flushArc();
			}
		}

		bool stitchedSame = true;
		while (stitchedSame)
		{
			stitchedSame = false;
			for (size_t i = 0; i < openBaseArcs.size() && !stitchedSame; i++)
			{
				if (openBaseArcs[i].pts.size() < 2) continue;
				for (size_t j = i + 1; j < openBaseArcs.size(); j++)
				{
					if (openBaseArcs[j].pts.size() < 2 || openBaseArcs[j].surfIdx != openBaseArcs[i].surfIdx)
						continue;
					auto& A = openBaseArcs[i].pts;
					auto& B = openBaseArcs[j].pts;
					const Vec3f a0 = A.front().ToVec3f(), a1 = A.back().ToVec3f();
					const Vec3f b0 = B.front().ToVec3f(), b1 = B.back().ToVec3f();
					float bestD = 10.0f;
					int mode = -1;
					auto checkM = [&](int m, const Vec3f& p, const Vec3f& q)
						{
							const float d = (q - p).length();
							if (d < bestD) { bestD = d; mode = m; }
						};
					checkM(0, a1, b0); checkM(1, a1, b1); checkM(2, a0, b1); checkM(3, a0, b0);
					if (mode != -1)
					{
						if (mode == 1) std::reverse(B.begin(), B.end());
						else if (mode == 2) std::swap(A, B);
						else if (mode == 3) std::reverse(A.begin(), A.end());
						A.insert(A.end(), B.begin(), B.end());
						B.clear();
						stitchedSame = true;
						break;
					}
				}
			}
		}

		std::vector<PureIntersectionLake> pureLakes;
		std::vector<bool> arcPaired(openBaseArcs.size(), false);
		for (size_t i = 0; i < openBaseArcs.size(); i++)
		{
			if (arcPaired[i] || openBaseArcs[i].pts.size() < 3) continue;
			for (size_t j = i + 1; j < openBaseArcs.size(); j++)
			{
				if (arcPaired[j] || openBaseArcs[j].pts.size() < 3 || openBaseArcs[j].surfIdx == openBaseArcs[i].surfIdx)
					continue;
				const auto& A = openBaseArcs[i].pts;
				const auto& B = openBaseArcs[j].pts;
				const float dSame = std::max((A.front().ToVec3f() - B.front().ToVec3f()).length(), (A.back().ToVec3f() - B.back().ToVec3f()).length());
				const float dFlip = std::max((A.front().ToVec3f() - B.back().ToVec3f()).length(), (A.back().ToVec3f() - B.front().ToVec3f()).length());
				if (std::min(dSame, dFlip) < 3.5f)
				{
					std::vector<float3> arcA = A;
					std::vector<float3> arcB = B;
					if (dFlip < dSame) std::reverse(arcB.begin(), arcB.end());

					std::vector<Vec2f> poly;
					poly.reserve(arcA.size() + arcB.size());
					for (const auto& p : arcA) poly.emplace_back(p.x, p.z);
					for (auto it = arcB.rbegin(); it != arcB.rend(); ++it) poly.emplace_back(it->x, it->z);
					double area2 = 0.0;
					for (size_t k = 0; k < poly.size(); k++)
					{
						const Vec2f& p0 = poly[k];
						const Vec2f& p1 = poly[(k + 1) % poly.size()];
						area2 += static_cast<double>(p0.x) * p1.y - static_cast<double>(p1.x) * p0.y;
					}
					const float areaMm2 = static_cast<float>(0.5 * std::abs(area2));
					if (areaMm2 >= 4.0f)
					{
						pureLakes.push_back({ std::move(arcA), std::move(arcB), areaMm2 });
						arcPaired[i] = true;
						arcPaired[j] = true;
						break;
					}
				}
			}
		}
		return pureLakes;
	}

	std::vector<std::vector<float3>> BuildPencilMillingChains(
		const std::vector<TracedFinishingIntersection>& tracedIntersections,
		const std::vector<std::pair<SceneObject*, bool>>& orderedSurfaces,
		const OffsetSurfaceMap& offsetSurfaces,
		const std::unordered_map<SceneObject*, size_t>& objToSurfIdx,
		const std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>>& surfAttachments,
		float yCenterMinWorld,
		float cutterRadiusMm,
		float halfX,
		float halfZ,
		float baseY,
		float minSafeTipY,
		float simplifyTol)
	{
		constexpr float cOutsideBlockMarginMm = 2.0f;
		const size_t numSurf = orderedSurfaces.size();
		std::vector<std::vector<float3>> activeGrooveChains;

		for (const auto& traced : tracedIntersections)
		{
			if (traced.pointsWorld.size() < 2)
				continue;

			auto itA = objToSurfIdx.find(traced.objA);
			auto itB = objToSurfIdx.find(traced.objB);
			const size_t idxA = (itA != objToSurfIdx.end()) ? itA->second : numSurf;
			const size_t idxB = (itB != objToSurfIdx.end()) ? itB->second : numSurf;
			const bool alongVA = (idxA < numSurf) ? orderedSurfaces[idxA].second : true;
			const bool alongVB = (idxB < numSurf) ? orderedSurfaces[idxB].second : true;

			const auto& clA = *offsetSurfaces.at(traced.objA);
			const auto& clB = *offsetSurfaces.at(traced.objB);

			std::vector<SurfaceCurveAttachment> otherAttsA, otherAttsB;
			if (auto attItA = surfAttachments.find(traced.objA); attItA != surfAttachments.end())
			{
				for (const auto& att : attItA->second)
					if (att.otherObj != nullptr && att.otherObj != traced.objB) otherAttsA.push_back(att);
			}
			if (auto attItB = surfAttachments.find(traced.objB); attItB != surfAttachments.end())
			{
				for (const auto& att : attItB->second)
					if (att.otherObj != nullptr && att.otherObj != traced.objA) otherAttsB.push_back(att);
			}

			auto isUVValidAgainstOther = [&](const GeometryMath::OffsetSurface& clSurf, bool alongVSurf,
				const std::vector<SurfaceCurveAttachment>& otherAtts, float uVal, float vVal) -> bool
				{
					if (otherAtts.empty()) return true;
					const float aVal = alongVSurf ? uVal : vVal;
					const float bVal = alongVSurf ? vVal : uVal;
					auto ivs = ComputeValidFlowIntervals(clSurf, alongVSurf, aVal, otherAtts, yCenterMinWorld);
					for (const auto& [b0, b1] : ivs)
						if (bVal >= b0 - 2e-3f && bVal <= b1 + 2e-3f) return true;
					return false;
				};

			auto evalBaseNormalY = [](SceneObject* o, float u, float v) -> float
				{
					auto e = GeometryMath::EvaluateParametric(o, u, v);
					Vec3f n = Vec3f::cross(e.du, e.dv);
					float len = n.length();
					return (len > 1e-6f) ? (n.y / len) : 1.0f;
				};

			std::vector<Vec3f> ptsWorld = traced.pointsWorld;
			std::vector<Vec4f> prms = traced.params;
			auto isValidPencilPt = [&](const Vec4f& prm, const Vec3f& pWorld, Vec3f& outTipMm) -> bool
				{
					Vec3f clMm = pWorld * cWorldToMm;
					outTipMm = Vec3f(clMm.x, clMm.y - cutterRadiusMm, clMm.z);
					if (outTipMm.y < baseY - 0.05f) return false;
					if (std::abs(outTipMm.x) > halfX + cutterRadiusMm + cOutsideBlockMarginMm ||
						std::abs(outTipMm.z) > halfZ + cutterRadiusMm + cOutsideBlockMarginMm) return false;
					if (evalBaseNormalY(traced.objA, prm.x, prm.y) < -0.02f ||
						evalBaseNormalY(traced.objB, prm.z, prm.w) < -0.02f) return false;
					if (!isUVValidAgainstOther(clA, alongVA, otherAttsA, prm.x, prm.y) ||
						!isUVValidAgainstOther(clB, alongVB, otherAttsB, prm.z, prm.w)) return false;
					outTipMm.y = std::max(outTipMm.y, minSafeTipY);
					return true;
				};

			if (traced.isClosedLoop && ptsWorld.size() > 3)
			{
				int invalidIdx = -1;
				for (size_t k = 0; k < ptsWorld.size(); k++)
				{
					Vec3f dummyTip{};
					if (!isValidPencilPt(prms[k], ptsWorld[k], dummyTip)) { invalidIdx = static_cast<int>(k); break; }
				}
				if (invalidIdx > 0)
				{
					ptsWorld.pop_back(); prms.pop_back();
					std::rotate(ptsWorld.begin(), ptsWorld.begin() + invalidIdx, ptsWorld.end());
					std::rotate(prms.begin(), prms.begin() + invalidIdx, prms.end());
					ptsWorld.push_back(ptsWorld.front()); prms.push_back(prms.front());
				}
			}

			std::vector<float3> currChain;
			auto flushChain = [&]()
				{
					if (currChain.size() >= 2)
					{
						float chainLen = 0.0f;
						for (size_t k = 1; k < currChain.size(); k++)
							chainLen += (currChain[k].ToVec3f() - currChain[k - 1].ToVec3f()).length();
						if (chainLen >= 3.0f)
							activeGrooveChains.push_back(SimplifyPolyline3DGougeSafe(currChain, std::max(simplifyTol, 0.025f)));
					}
					currChain.clear();
				};

			for (size_t k = 0; k < ptsWorld.size(); k++)
			{
				Vec3f tipMm{};
				if (isValidPencilPt(prms[k], ptsWorld[k], tipMm))
					currChain.emplace_back(tipMm.x, tipMm.y, tipMm.z);
				else
					flushChain();
			}
			flushChain();
		}

		bool stitchedAny = true;
		while (stitchedAny)
		{
			stitchedAny = false;
			for (size_t i = 0; i < activeGrooveChains.size() && !stitchedAny; i++)
			{
				if (activeGrooveChains[i].size() < 2) continue;
				for (size_t j = i + 1; j < activeGrooveChains.size(); j++)
				{
					if (activeGrooveChains[j].size() < 2) continue;
					auto& A = activeGrooveChains[i];
					auto& B = activeGrooveChains[j];
					const Vec3f a0 = A.front().ToVec3f(), a1 = A.back().ToVec3f();
					const Vec3f b0 = B.front().ToVec3f(), b1 = B.back().ToVec3f();
					float bestGap = 2.0f;
					int mode = -1;
					auto testMode = [&](int m, const Vec3f& p, const Vec3f& q)
						{
							const float d = (q - p).length();
							if (d < bestGap) { bestGap = d; mode = m; }
						};
					testMode(0, a1, b0); testMode(1, a1, b1); testMode(2, a0, b1); testMode(3, a0, b0);
					if (mode != -1)
					{
						if (mode == 1) std::reverse(B.begin(), B.end());
						else if (mode == 2) std::swap(A, B);
						else if (mode == 3) std::reverse(A.begin(), A.end());
						A.insert(A.end(), B.begin(), B.end());
						B.clear();
						stitchedAny = true;
						break;
					}
				}
			}
		}
		activeGrooveChains.erase(
			std::remove_if(activeGrooveChains.begin(), activeGrooveChains.end(),
				[](const auto& c) { return c.size() < 2; }),
			activeGrooveChains.end());

		return activeGrooveChains;
	}
	void EmitAheadTransition(FinishingPathWriter& writer, const std::vector<float3>& pts)
	{
		std::vector<float3> ahead;
		for (size_t k = 1; k < pts.size() && ahead.size() < 80; k++)
		{
			ahead.push_back(pts[k]);
			if (std::hypot(pts[k].x - pts[0].x, pts[k].z - pts[0].z) >= 3.0f)
				break;
		}
		writer.TransitionSafeCl(pts.front(), &ahead);
	}

	void AppendSurfaceZigzagPasses(
		const std::vector<std::pair<SceneObject*, bool>>& orderedSurfaces,
		const OffsetSurfaceMap& offsetSurfaces,
		const std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>>& surfAttachments,
		const FinishingParameters& params,
		float R,
		float baseY,
		float minSafeTipY,
		float yCenterMinWorld,
		float halfX,
		float halfZ,
		float safeY,
		float cOutsideBlockMarginMm,
		FinishingPathWriter& writer)
	{
		const float scallopH = std::clamp(params.scallopHeight, 0.002f, 0.5f);
		const float minStepMm = std::clamp(params.minStepover, 0.10f, 5.0f);
		const float maxStepMm = std::clamp(params.maxStepover, minStepMm, 6.0f);
		const float curvWeight = std::max(0.0f, params.curvatureWeight);
		const float baseFlatWidth = 2.0f * std::sqrt(2.0f * R * scallopH);
		const float simplifyTol = std::max(0.005f, params.simplifyTolerance);
		const size_t numSurf = orderedSurfaces.size();
		auto& waypoints = writer.waypoints;
		auto appendSafePoint = [&](const float3& pt) { writer.AppendSafePoint(pt); };
		auto evaluateExteriorAccess = [&](const Vec3f& dest) { return writer.EvaluateExteriorAccess(dest); };
		auto transitionSafeCl = [&](const std::vector<float3>& pts) { EmitAheadTransition(writer, pts); };
		constexpr float cMinSafeHorizontalSlope = FinishingPathWriter::cMinSafeHorizontalSlope;

	for (size_t surfIdx = 0; surfIdx < numSurf; surfIdx++)
	{

		SceneObject* obj = orderedSurfaces[surfIdx].first;
		const bool alongV = orderedSurfaces[surfIdx].second;
		const auto& clSelf = *offsetSurfaces.at(obj);
		static const std::vector<SurfaceCurveAttachment> cEmptyAtts;
		auto attIt = surfAttachments.find(obj);
		const auto& attachments = (attIt != surfAttachments.end()) ? attIt->second : cEmptyAtts;

		const float maxU = clSelf.GetMaxU();
		const float maxV = clSelf.GetMaxV();
		const bool wrapU = clSelf.IsSeamConnected(true, 0.5f * maxV);
		const bool wrapV = clSelf.IsSeamConnected(false, 0.5f * maxU);
		const float maxAlpha = alongV ? maxU : maxV;
		const float maxBeta = alongV ? maxV : maxU;
		const bool wrapAlpha = alongV ? wrapU : wrapV;
		const bool wrapBeta = alongV ? wrapV : wrapU;

		struct SurfSample
		{
			Vec3f posMm{};
			Vec3f duMm{};
			Vec3f dvMm{};
			Vec3f normal{};
			Vec3f tipMm{};
		};

		auto evalAtAB = [&](float alpha, float beta) -> SurfSample
			{
				const float a = WrapOrClampParam(alpha, maxAlpha, wrapAlpha);
				const float b = WrapOrClampParam(beta, maxBeta, wrapBeta);
				const float u = alongV ? a : b;
				const float v = alongV ? b : a;

				auto baseEval = GeometryMath::EvaluateParametric(obj, u, v);
				SurfSample s{};
				s.posMm = baseEval.p * cWorldToMm;
				s.duMm = baseEval.du * cWorldToMm;
				s.dvMm = baseEval.dv * cWorldToMm;

				Vec3f rawN = Vec3f::cross(s.duMm, s.dvMm);
				float nLen = rawN.length();
				if (nLen > 1e-6f)
				{
					s.normal = rawN / nLen;
					s.tipMm = s.posMm + s.normal * R - Vec3f(0.0f, R, 0.0f);
				}
				else
				{
					s.normal = Vec3f(0.0f, 1.0f, 0.0f);
					s.tipMm = s.posMm - Vec3f(0.0f, R, 0.0f);
				}
				return s;
			};

		auto isPointValid = [&](float alpha, float beta, SurfSample* outSample = nullptr) -> bool
			{
				SurfSample s = evalAtAB(alpha, beta);
				if (outSample)
					*outSample = s;

				// 1. Contact point must lie on or above the base plane
				if (s.posMm.y < baseY - 0.02f)
					return false;

				// 2. Outward normal must point upward (excludes vertical/undercut closure walls like hull segU=2 where n.y == 0)
				if (s.normal.y < 0.005f)
					return false;

				// 3. Cutter tip must stay above the base plane
				if (s.tipMm.y < baseY - 0.01f)
					return false;

				// 4. Cutter tip must stay within the machinable block area
				if (std::abs(s.tipMm.x) > halfX + R + cOutsideBlockMarginMm ||
					std::abs(s.tipMm.z) > halfZ + R + cOutsideBlockMarginMm)
					return false;


				return true;
			};

		// Adaptive cross-parameter step based on surface derivative & transverse curvature
		auto computeAdaptiveStepAlpha = [&](float alpha) -> float
			{
				constexpr int cProbeBeta = 36;
				const float deltaA = std::max(1e-4f, 0.001f * maxAlpha);
				const float patchMinA = 0.0f, patchMaxA = maxAlpha;
				float minParamStep = maxAlpha;
				bool anyValid = false;

				for (int i = 0; i <= cProbeBeta; i++)
				{
					const float beta = (maxBeta * i) / cProbeBeta;
					SurfSample s0;
					if (!isPointValid(alpha, beta, &s0))
						continue;

					anyValid = true;
					const Vec3f& Pb = alongV ? s0.dvMm : s0.duMm;
					const Vec3f& Pa = alongV ? s0.duMm : s0.dvMm;
					const float pbLen = Pb.length();
					const float paLenSq = Pa.length_sqr();
					const float crossLen = Vec3f::cross(Pb, Pa).length();
					const float lPerp = (pbLen > 1e-4f) ? (crossLen / pbLen) : Pa.length();
					if (lPerp < 1e-3f)
						continue;

					const bool useWrapDiff = wrapAlpha;
					float aMinus = useWrapDiff ? WrapOrClampParam(alpha - deltaA, maxAlpha, true) : std::clamp(alpha - deltaA, patchMinA, patchMaxA);
					float aPlus = useWrapDiff ? WrapOrClampParam(alpha + deltaA, maxAlpha, true) : std::clamp(alpha + deltaA, patchMinA, patchMaxA);
					float actualSpan = (useWrapDiff || (alpha - deltaA >= patchMinA && alpha + deltaA <= patchMaxA)) ? (2.0f * deltaA) : (aPlus - aMinus);

					float effectiveLPerp = lPerp;
					float absKappaPerp = 0.0f;
					if (actualSpan > 1e-5f && paLenSq > 1e-5f)
					{
						SurfSample sMinus = evalAtAB(aMinus, beta);
						SurfSample sPlus = evalAtAB(aPlus, beta);
						Vec3f dNormalDa = (sPlus.normal - sMinus.normal) * (1.0f / actualSpan);
						absKappaPerp = std::abs(Vec3f::dot(dNormalDa, Pa)) / paLenSq;

						Vec3f dCenterDa = ((sPlus.tipMm - sMinus.tipMm) * (1.0f / actualSpan));
						float dCenterPerp = (pbLen > 1e-4f) ? (Vec3f::cross(Pb, dCenterDa).length() / pbLen) : dCenterDa.length();
						effectiveLPerp = std::max(lPerp, 0.65f * dCenterPerp);
					}

					float denom = std::max(0.20f, 1.0f + curvWeight * R * absKappaPerp);
					float localStripMm = baseFlatWidth / std::sqrt(denom);
					localStripMm = std::clamp(localStripMm, minStepMm, maxStepMm);
					minParamStep = std::min(minParamStep, localStripMm / effectiveLPerp);
				}

				if (!anyValid)
				{
					float probeA = std::clamp(alpha + 0.02f * maxAlpha, patchMinA, patchMaxA);
					SurfSample sMid = evalAtAB(probeA, 0.5f * maxBeta);
					const Vec3f& Pa = alongV ? sMid.duMm : sMid.dvMm;
					float paLen = std::max(10.0f, Pa.length());
					return std::clamp(maxStepMm / paLen, maxAlpha / 400.0f, maxAlpha / 35.0f);
				}

				return std::clamp(minParamStep, maxAlpha / 500.0f, maxAlpha / 18.0f);
			};

		std::vector<float> alphaValues;
		alphaValues.reserve(256);
		{
			float currA = 0.0f;
			while (currA < maxAlpha - 1e-5f)
			{
				alphaValues.push_back(currA);
				float da = computeAdaptiveStepAlpha(currA);
				if (currA + da >= maxAlpha)
					break;
				currA += da;
			}
			if (!wrapAlpha)
				alphaValues.push_back(maxAlpha);
		}

		// Extract valid trimmed pass segments for any cross-parameter alpha:
		// starts from exact offset-surface intersection intervals (surface-surface & surface-base)
		// and validates against upward normal + non-self-blocking ITO safety.
		auto buildPassSegmentsForAlpha = [&](float alpha) -> std::vector<FinishingIsoPassSegment>
			{
				auto clIntervals = ComputeValidFlowIntervals(clSelf, alongV, alpha, attachments, yCenterMinWorld);
				if (clIntervals.empty())
					return {};

				std::vector<std::pair<float, float>> rawValidIntervals;
				for (const auto& [segB0, segB1] : clIntervals)
				{
					const float spanB = segB1 - segB0;
					if (spanB < 1e-5f)
						continue;

					SurfSample sA = evalAtAB(alpha, segB0);
					SurfSample sB = evalAtAB(alpha, 0.5f * (segB0 + segB1));
					SurfSample sC = evalAtAB(alpha, segB1);
					const float maxDerivMm = std::max({
						(alongV ? sA.dvMm : sA.duMm).length(),
						(alongV ? sB.dvMm : sB.duMm).length(),
						(alongV ? sC.dvMm : sC.duMm).length(),
						1.0f
						});
					const int numSteps = std::clamp(static_cast<int>(std::ceil(spanB * maxDerivMm / 0.20f)), 6, 500);

					bool prevValid = isPointValid(alpha, segB0);
					float prevB = segB0;
					float activeStartB = prevValid ? segB0 : -1.0f;

					auto bisectBoundary = [&](float bValid, float bInvalid) -> float
						{
							float lo = bValid;
							float hi = bInvalid;
							for (int iter = 0; iter < 10; iter++)
							{
								float mid = 0.5f * (lo + hi);
								if (isPointValid(alpha, mid))
									lo = mid;
								else
									hi = mid;
							}
							return lo;
						};

					for (int s = 1; s <= numSteps; s++)
					{
						float currB = segB0 + (spanB * s) / numSteps;
						bool currValid = isPointValid(alpha, currB);

						if (!prevValid && currValid)
						{
							activeStartB = bisectBoundary(currB, prevB);
						}
						else if (prevValid && !currValid)
						{
							float endB = bisectBoundary(prevB, currB);
							if (endB > activeStartB + 1e-5f)
								rawValidIntervals.push_back({ activeStartB, endB });
							activeStartB = -1.0f;
						}

						prevB = currB;
						prevValid = currValid;
					}

					if (prevValid && activeStartB >= 0.0f && segB1 > activeStartB + 1e-5f)
						rawValidIntervals.push_back({ activeStartB, segB1 });
				}

				if (rawValidIntervals.empty())
					return {};

				// Merge consecutive intervals touching at a valid C1 point
				std::vector<std::pair<float, float>> mergedIntervals;
				for (const auto& iv : rawValidIntervals)
				{
					bool canMerge = false;
					if (!mergedIntervals.empty() && std::abs(iv.first - mergedIntervals.back().second) < 3e-4f)
					{
						const float bJoin = 0.5f * (iv.first + mergedIntervals.back().second);
						if (isPointValid(alpha, bJoin))
						{
							SurfSample sL = evalAtAB(alpha, std::max(0.0f, mergedIntervals.back().second - 2e-4f));
							SurfSample sR = evalAtAB(alpha, std::min(maxBeta, iv.first + 2e-4f));
							if (Vec3f::dot(sL.normal, sR.normal) >= 0.85f &&
								(sL.tipMm - sR.tipMm).length_sqr() < 0.25f * 0.25f)
							{
								canMerge = true;
							}
						}
					}

					if (canMerge)
						mergedIntervals.back().second = iv.second;
					else
						mergedIntervals.push_back(iv);
				}

				// Merge wrapped intervals across periodic seam beta = 0 == maxBeta
				if (wrapBeta && mergedIntervals.size() >= 2)
				{
					if (mergedIntervals.front().first < 1e-3f && mergedIntervals.back().second > maxBeta - 1e-3f)
					{
						SurfSample sZero = evalAtAB(alpha, mergedIntervals.front().first);
						SurfSample sMax = evalAtAB(alpha, mergedIntervals.back().second);
						if ((sZero.tipMm - sMax.tipMm).length_sqr() < 0.05f * 0.05f)
						{
							mergedIntervals.back().second = maxBeta + mergedIntervals.front().second;
							mergedIntervals.erase(mergedIntervals.begin());
						}
					}
				}

				std::vector<FinishingIsoPassSegment> rowSegs;
				for (const auto& [b0, b1] : mergedIntervals)
				{
					const float spanB = b1 - b0;
					SurfSample sStart = evalAtAB(alpha, b0);
					SurfSample sMid = evalAtAB(alpha, 0.5f * (b0 + b1));
					SurfSample sEnd = evalAtAB(alpha, b1);
					const float chordEstimateMm = (sEnd.tipMm - sStart.tipMm).length() + (sMid.tipMm - sStart.tipMm).length();
					if (chordEstimateMm < 0.25f)
						continue;

					const int sampleSteps = std::clamp(static_cast<int>(std::ceil(chordEstimateMm / 0.18f)), 4, 500);
					std::vector<float3> rawPts;
					rawPts.reserve(sampleSteps + 1);
					for (int s = 0; s <= sampleSteps; s++)
					{
						float b = b0 + (spanB * s) / sampleSteps;
						SurfSample smp = evalAtAB(alpha, b);
						float clampedTipY = std::max(smp.tipMm.y, minSafeTipY);
						rawPts.emplace_back(smp.tipMm.x, clampedTipY, smp.tipMm.z);
					}

					FinishingIsoPassSegment seg;
					seg.alpha = alpha;
					seg.betaStart = b0;
					seg.betaEnd = b1;
					seg.pts = SimplifyPolyline3DGougeSafe(rawPts, simplifyTol);
					if (seg.pts.size() >= 2)
						rowSegs.push_back(std::move(seg));
				}

				return rowSegs;
			};

		std::vector<std::vector<FinishingIsoPassSegment>> passRows(alphaValues.size());
#pragma omp parallel for schedule(dynamic, 1)
		for (int j = 0; j < static_cast<int>(alphaValues.size()); j++)
			passRows[j] = buildPassSegmentsForAlpha(alphaValues[j]);

		// Refine cross-parameter boundaries where valid passes start/stop or jump sharply
		for (int refPass = 0; refPass < 2; refPass++)
		{
			std::vector<float> nextAlpha;
			std::vector<std::vector<FinishingIsoPassSegment>> nextRows;
			nextAlpha.reserve(alphaValues.size() * 2);
			nextRows.reserve(passRows.size() * 2);

			auto totalSpanBeta = [](const std::vector<FinishingIsoPassSegment>& row) -> float
				{
					float sum = 0.0f;
					for (const auto& s : row) sum += (s.betaEnd - s.betaStart);
					return sum;
				};

			bool addedAny = false;
			for (size_t j = 0; j < alphaValues.size(); j++)
			{
				nextAlpha.push_back(alphaValues[j]);
				nextRows.push_back(std::move(passRows[j]));

				if (j + 1 < alphaValues.size())
				{
					const bool emptyA = nextRows.back().empty();
					const bool emptyB = passRows[j + 1].empty();
					const float spanA = totalSpanBeta(nextRows.back());
					const float spanB = totalSpanBeta(passRows[j + 1]);
					const float sharpClipThreshold = std::min(0.12f * maxBeta, 0.9f);
					const bool sharpClip = (!emptyA && !emptyB && std::abs(spanA - spanB) > sharpClipThreshold);
					if ((emptyA != emptyB) || sharpClip)
					{
						const float aMid = 0.5f * (alphaValues[j] + alphaValues[j + 1]);
						if (std::abs(alphaValues[j + 1] - alphaValues[j]) > maxAlpha / 400.0f)
						{
							auto midSegs = buildPassSegmentsForAlpha(aMid);
							if (!midSegs.empty())
							{
								nextAlpha.push_back(aMid);
								nextRows.push_back(std::move(midSegs));
								addedAny = true;
							}
						}
					}
				}
			}

			alphaValues = std::move(nextAlpha);
			passRows = std::move(nextRows);
			if (!addedAny)
				break;
		}

		// Rotate cyclic alpha rows past an empty gap so periodic surfaces have unbroken zigzags
		if (wrapAlpha && passRows.size() >= 4 && !passRows.front().empty() && !passRows.back().empty())
		{
			std::optional<size_t> gapIdx;
			for (size_t j = 0; j < passRows.size(); j++)
			{
				if (passRows[j].empty())
				{
					gapIdx = (j + 1) % passRows.size();
					break;
				}
			}
			if (gapIdx && *gapIdx > 0)
				std::rotate(passRows.begin(), passRows.begin() + *gapIdx, passRows.end());
		}

		// Group pass segments across consecutive rows into continuous zigzag chains
		struct ZigzagChain
		{
			std::vector<FinishingIsoPassSegment> segments;
		};
		std::vector<ZigzagChain> chains;
		std::vector<size_t> prevRowChainIds;

		for (size_t j = 0; j < passRows.size(); j++)
		{
			auto& row = passRows[j];
			if (row.empty())
			{
				prevRowChainIds.clear();
				continue;
			}

			std::vector<size_t> currRowChainIds(row.size(), std::numeric_limits<size_t>::max());
			std::vector<bool> prevUsed(prevRowChainIds.size(), false);

			for (size_t sIdx = 0; sIdx < row.size(); sIdx++)
			{
				const auto& seg = row[sIdx];
				const Vec3f segStart = seg.pts.front().ToVec3f();
				const Vec3f segEnd = seg.pts.back().ToVec3f();

				float bestScore = std::numeric_limits<float>::max();
				std::optional<size_t> bestPrevIdx;

				if (j > 0 && !passRows[j - 1].empty())
				{
					for (size_t pIdx = 0; pIdx < passRows[j - 1].size(); pIdx++)
					{
						if (prevUsed[pIdx])
							continue;

						const auto& prevSeg = passRows[j - 1][pIdx];
						const float overlapB = std::min(seg.betaEnd, prevSeg.betaEnd) - std::max(seg.betaStart, prevSeg.betaStart);
						const float dStart = (segStart - prevSeg.pts.front().ToVec3f()).length();
						const float dEnd = (segEnd - prevSeg.pts.back().ToVec3f()).length();
						const float minEndDist = std::min(dStart, dEnd);
						const float avgEndDist = 0.5f * (dStart + dEnd);

						const bool hasParamOverlap = (overlapB > -0.06f * maxBeta);
						if (hasParamOverlap && minEndDist <= 28.0f && avgEndDist <= 40.0f)
						{
							const float chainBonus = 0.35f * static_cast<float>(chains[prevRowChainIds[pIdx]].segments.size());
							const float score = avgEndDist - 6.0f * overlapB - chainBonus;
							if (score < bestScore)
							{
								bestScore = score;
								bestPrevIdx = pIdx;
							}
						}
					}
				}

				if (bestPrevIdx)
				{
					prevUsed[*bestPrevIdx] = true;
					size_t chainId = prevRowChainIds[*bestPrevIdx];
					currRowChainIds[sIdx] = chainId;
					chains[chainId].segments.push_back(seg);
				}
				else
				{
					size_t newChainId = chains.size();
					currRowChainIds[sIdx] = newChainId;
					chains.push_back({ { seg } });
				}
			}

			prevRowChainIds = std::move(currRowChainIds);
		}

		// Filter out tiny narrow sliver chains (e.g. narrow crease pockets already swept by Step 8 pencil milling)
		std::vector<ZigzagChain> validChains;
		validChains.reserve(chains.size());
		std::vector<float> chainLengths;
		chainLengths.reserve(chains.size());
		for (auto& c : chains)
		{
			if (c.segments.empty())
				continue;
			float totalLen = 0.0f;
			float maxSegLen = 0.0f;
			for (const auto& sg : c.segments)
			{
				float segLen = 0.0f;
				for (size_t k = 1; k < sg.pts.size(); k++)
					segLen += (sg.pts[k].ToVec3f() - sg.pts[k - 1].ToVec3f()).length();
				totalLen += segLen;
				maxSegLen = std::max(maxSegLen, segLen);
			}
			if ((totalLen < 12.0f && c.segments.size() <= 4) ||
				(c.segments.size() <= 2 && maxSegLen < 8.0f))
				continue;
			chainLengths.push_back(totalLen);
			validChains.push_back(std::move(c));
		}

		// Compute globally optimal chain visit order & entry orientation (exact Held-Karp DP for N <= 8)
		// to eliminate back-and-forth air flights across split cylinders (e.g. wave2) and between surfaces.
		struct ChainPlanItem
		{
			size_t chainIdx = 0;
			bool reverseRowOrder = false;
			bool initForwardDir = true;
		};
		std::vector<ChainPlanItem> chainPlan;
		const size_t numValidChains = validChains.size();
		if (numValidChains > 0)
		{
			auto getChainEndpoints = [&](size_t cIdx, int orient, Vec3f& outEntry, Vec3f& outExit)
				{
					const auto& segs = validChains[cIdx].segments;
					const bool rev = (orient >= 2);
					const bool initFwd = ((orient % 2) == 0);
					const auto& firstSeg = rev ? segs.back() : segs.front();
					const auto& lastSeg = rev ? segs.front() : segs.back();
					const bool lastFwd = ((segs.size() % 2) == 1) ? initFwd : !initFwd;
					outEntry = (initFwd ? firstSeg.pts.front() : firstSeg.pts.back()).ToVec3f();
					outExit = (lastFwd ? lastSeg.pts.back() : lastSeg.pts.front()).ToVec3f();
				};

			auto chainTransferCost = [&](const Vec3f& fromPt, const Vec3f& toPt, float liftPenalty) -> float
				{
					const float dist = (toPt - fromPt).length();
					if (liftPenalty > 50.0f && dist <= 15.0f)
						return dist;
					if (dist <= 1.5f)
						return dist;

					return dist + liftPenalty;
				};

			if (numValidChains <= 8)
			{
				const size_t N = numValidChains;
				std::vector<std::array<Vec3f, 4>> entryPts(N), exitPts(N);
				for (size_t c = 0; c < N; c++)
				{
					for (int o = 0; o < 4; o++)
						getChainEndpoints(c, o, entryPts[c][o], exitPts[c][o]);
				}

				const size_t numStates = (static_cast<size_t>(1) << N) * N * 4;
				constexpr float cInf = std::numeric_limits<float>::max() * 0.25f;
				std::vector<float> dp(numStates, cInf);
				std::vector<int> parent(numStates, -1);

				auto stateIdx = [&](size_t mask, size_t c, int o) -> size_t
					{
						return (mask * N + c) * 4 + static_cast<size_t>(o);
					};

				for (size_t c = 0; c < N; c++)
				{
					const size_t m = static_cast<size_t>(1) << c;
					for (int o = 0; o < 4; o++)
					{
						if (waypoints.empty())
							dp[stateIdx(m, c, o)] = evaluateExteriorAccess(entryPts[c][o]).score;
						else
							dp[stateIdx(m, c, o)] = chainTransferCost(waypoints.back().ToVec3f(), entryPts[c][o], 8.0f);
					}
				}

				const size_t fullMask = (static_cast<size_t>(1) << N) - 1;
				for (size_t mask = 1; mask <= fullMask; mask++)
				{
					for (size_t c = 0; c < N; c++)
					{
						if (!(mask & (static_cast<size_t>(1) << c)))
							continue;
						for (int o = 0; o < 4; o++)
						{
							const float curCost = dp[stateIdx(mask, c, o)];
							if (curCost >= cInf * 0.5f)
								continue;
							for (size_t nc = 0; nc < N; nc++)
							{
								if (mask & (static_cast<size_t>(1) << nc))
									continue;
								const size_t nMask = mask | (static_cast<size_t>(1) << nc);
								for (int no = 0; no < 4; no++)
								{
									const float candCost = curCost + chainTransferCost(exitPts[c][o], entryPts[nc][no], 100.0f);
									const size_t ns = stateIdx(nMask, nc, no);
									if (candCost < dp[ns])
									{
										dp[ns] = candCost;
										parent[ns] = static_cast<int>(c * 4 + static_cast<size_t>(o));
									}
								}
							}
						}
					}
				}

				float bestTotal = cInf;
				size_t bestC = 0;
				int bestO = 0;
				for (size_t c = 0; c < N; c++)
				{
					for (int o = 0; o < 4; o++)
					{
						const float val = dp[stateIdx(fullMask, c, o)];
						if (val < bestTotal)
						{
							bestTotal = val;
							bestC = c;
							bestO = o;
						}
					}
				}

				chainPlan.resize(N);
				size_t curMask = fullMask;
				size_t curC = bestC;
				int curO = bestO;
				for (size_t step = N; step > 0; step--)
				{
					chainPlan[step - 1] = { curC, (curO >= 2), ((curO % 2) == 0) };
					const int prevPacked = parent[stateIdx(curMask, curC, curO)];
					curMask ^= (static_cast<size_t>(1) << curC);
					if (prevPacked >= 0)
					{
						curC = static_cast<size_t>(prevPacked / 4);
						curO = prevPacked % 4;
					}
				}
			}
			else
			{
				std::vector<bool> chainVisited(numValidChains, false);
				Vec3f curPos = waypoints.empty() ? Vec3f(0.0f, safeY, 0.0f) : waypoints.back().ToVec3f();
				for (size_t visitedCount = 0; visitedCount < numValidChains; visitedCount++)
				{
					size_t bestC = 0;
					int bestO = 0;
					float bestCost = std::numeric_limits<float>::max();
					Vec3f bestExit{};
					for (size_t c = 0; c < numValidChains; c++)
					{
						if (chainVisited[c])
							continue;
						for (int o = 0; o < 4; o++)
						{
							Vec3f en{}, ex{};
							getChainEndpoints(c, o, en, ex);
							const float cost = (visitedCount == 0 && waypoints.empty())
								? evaluateExteriorAccess(en).score
								: chainTransferCost(curPos, en, visitedCount == 0 ? 35.0f : 100.0f);
							if (cost < bestCost)
							{
								bestCost = cost;
								bestC = c;
								bestO = o;
								bestExit = ex;
							}
						}
					}
					chainVisited[bestC] = true;
					chainPlan.push_back({ bestC, (bestO >= 2), ((bestO % 2) == 0) });
					curPos = bestExit;
				}
			}
		}

		for (const auto& planItem : chainPlan)
		{
			auto& chain = validChains[planItem.chainIdx];
			if (planItem.reverseRowOrder)
				std::reverse(chain.segments.begin(), chain.segments.end());

			bool forwardDir = planItem.initForwardDir;
			float actualPrevEndBeta = 0.0f;
			float actualPrevAlpha = 0.0f;

			for (size_t sIdx = 0; sIdx < chain.segments.size(); sIdx++)
			{
				const auto& seg = chain.segments[sIdx];
				std::vector<float3> segPts = seg.pts;
				if (!forwardDir)
					std::reverse(segPts.begin(), segPts.end());

				const float actualCurrStartBeta = forwardDir ? seg.betaStart : seg.betaEnd;
				const float actualCurrEndBeta = forwardDir ? seg.betaEnd : seg.betaStart;
				const float actualCurrAlpha = seg.alpha;

				if (sIdx == 0)
				{
					const Vec3f nextStart = segPts.front().ToVec3f();
					const float chainGap = waypoints.empty() ? 1e9f : (nextStart - waypoints.back().ToVec3f()).length();
					if (!waypoints.empty() && chainGap <= 1.5f)
					{
						appendSafePoint(segPts.front());
					}

					else
					{
						transitionSafeCl(segPts);
					}
				}
				else
				{
					const Vec3f prevEnd = waypoints.back().ToVec3f();
					const Vec3f nextStart = segPts.front().ToVec3f();
					const float linkDist = (nextStart - prevEnd).length();

					// Pure intersection-based turnaround linking:
					// only connect straight across tiny <= 0.70 mm gaps; larger links hug the boundary or lift cleanly.
					const bool straightLineSafe = (linkDist <= 0.70f);

					const bool canTryBoundaryTrace =
						(linkDist > 0.70f && linkDist <= 45.0f) &&
						std::abs(actualCurrStartBeta - actualPrevEndBeta) < 0.55f * maxBeta &&
						std::abs(actualCurrAlpha - actualPrevAlpha) < 0.25f * maxAlpha;

					if (linkDist <= 0.70f)
					{
						appendSafePoint(segPts.front());
					}
					else if (canTryBoundaryTrace)
					{
						bool boundaryTraceSafe = true;
						std::vector<float3> linkPts;
						const int linkSteps = std::clamp(static_cast<int>(std::ceil(linkDist / 0.50f)), 2, 90);
						const bool bothOnBase =
							(prevEnd.y <= baseY + 0.65f) && (nextStart.y <= baseY + 0.65f);
						const float betaDelta = std::abs(actualCurrStartBeta - actualPrevEndBeta);
						const float maxBetaSnap = bothOnBase
							? std::max(0.18f * maxBeta, betaDelta * 0.75f)
							: std::max(0.05f * maxBeta, betaDelta * 0.85f);
						const float maxAllowedLinkY = std::max(prevEnd.y, nextStart.y) + (bothOnBase ? 0.65f : 2.5f);
						for (int ls = 1; ls < linkSteps; ls++)
						{
							const float t = static_cast<float>(ls) / linkSteps;
							const float bLinear = std::lerp(actualPrevEndBeta, actualCurrStartBeta, t);
							float bInterp = bLinear;
							const float aInterp = std::lerp(actualPrevAlpha, actualCurrAlpha, t);
							{
								// Snap turnaround to the exact trim boundary (base plane OR surface-surface intersection)
								// so the cutter rolls smoothly along the intersection instead of lifting or cutting across the surface.
								auto ivs = ComputeValidFlowIntervals(clSelf, alongV, aInterp, attachments, yCenterMinWorld);
								float bestSnapDist = maxBetaSnap;
								float bestSnapBeta = bInterp;
								for (const auto& [b0, b1] : ivs)
								{
									const float d0 = std::abs(b0 - bLinear);
									if (d0 < bestSnapDist && evalAtAB(aInterp, b0).tipMm.y <= maxAllowedLinkY)
									{
										bestSnapDist = d0;
										bestSnapBeta = b0;
									}
									const float d1 = std::abs(b1 - bLinear);
									if (d1 < bestSnapDist && evalAtAB(aInterp, b1).tipMm.y <= maxAllowedLinkY)
									{
										bestSnapDist = d1;
										bestSnapBeta = b1;
									}
								}
								bInterp = bestSnapBeta;
							}
							SurfSample smp = evalAtAB(aInterp, bInterp);
							const float safeLinkY = std::max(smp.tipMm.y, minSafeTipY);
							if (safeLinkY > maxAllowedLinkY)
							{
								boundaryTraceSafe = false;
								break;
							}
							linkPts.emplace_back(smp.tipMm.x, safeLinkY, smp.tipMm.z);
						}

						if (boundaryTraceSafe)
						{
							for (const auto& lp : linkPts)
								appendSafePoint(lp);
							appendSafePoint(segPts.front());
						}
						else if (straightLineSafe)
						{
							appendSafePoint(segPts.front());
						}
						else
						{
							transitionSafeCl(segPts);
						}
					}
					else if (straightLineSafe)
					{
						appendSafePoint(segPts.front());
					}
					else
					{
						transitionSafeCl(segPts);
					}
				}

				for (size_t k = 1; k < segPts.size(); k++)
					appendSafePoint(segPts[k]);

				actualPrevEndBeta = actualCurrEndBeta;
				actualPrevAlpha = actualCurrAlpha;
				forwardDir = !forwardDir;
			}
		}

	}

	}

	void AppendEnclosedLakePasses(
		const std::vector<std::pair<SceneObject*, bool>>& orderedSurfaces,
		const OffsetSurfaceMap& offsetSurfaces,
		const std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>>& surfAttachments,
		const FinishingParameters& params,
		float R,
		float minSafeTipY,
		float yCenterMinWorld,
		float halfX,
		float halfZ,
		FinishingPathWriter& writer)
	{
		const float scallopH = std::clamp(params.scallopHeight, 0.002f, 0.5f);
		const float minStepMm = std::clamp(params.minStepover, 0.10f, 5.0f);
		const float maxStepMm = std::clamp(params.maxStepover, minStepMm, 6.0f);
		const float simplifyTol = std::max(0.005f, params.simplifyTolerance);
		auto& waypoints = writer.waypoints;
		auto appendSafePoint = [&](const float3& pt) { writer.AppendSafePoint(pt); };
		auto transitionSafeClPoint = [&](const float3& pt) { writer.TransitionSafeCl(pt); };

	// 7. Clear enclosed flat base lakes purely from trimmed OffsetSurface x HorizontalPlaneSurface contours
	{
		std::vector<PureIntersectionLake> pureLakes = ExtractPureIntersectionLakes(
			orderedSurfaces, offsetSurfaces, surfAttachments, yCenterMinWorld, halfX, halfZ, minSafeTipY);

		const float flatStep = std::clamp(
			2.0f * std::sqrt(std::max(0.01f, 2.0f * R * scallopH - scallopH * scallopH)),
			minStepMm,
			maxStepMm);

		auto resampleArcUniform = [](const std::vector<float3>& arc, int numSamples) -> std::vector<Vec3f>
			{
				std::vector<float> cumLen(arc.size(), 0.0f);
				for (size_t i = 1; i < arc.size(); i++)
					cumLen[i] = cumLen[i - 1] + (arc[i].ToVec3f() - arc[i - 1].ToVec3f()).length();
				const float totalLen = std::max(1e-5f, cumLen.back());

				std::vector<Vec3f> out(numSamples + 1);
				size_t segIdx = 0;
				for (int s = 0; s <= numSamples; s++)
				{
					const float target = (totalLen * s) / numSamples;
					while (segIdx + 1 < arc.size() && cumLen[segIdx + 1] < target)
						segIdx++;
					if (segIdx + 1 >= arc.size())
					{
						out[s] = arc.back().ToVec3f();
					}
					else
					{
						const float segLen = cumLen[segIdx + 1] - cumLen[segIdx];
						const float t = (segLen > 1e-6f) ? std::clamp((target - cumLen[segIdx]) / segLen, 0.0f, 1.0f) : 0.0f;
						out[s] = Vec3f::lerp(arc[segIdx].ToVec3f(), arc[segIdx + 1].ToVec3f(), t);
					}
				}
				return out;
			};

		auto polylineLen = [](const std::vector<float3>& pts) -> float
			{
				float s = 0.0f;
				for (size_t k = 1; k < pts.size(); k++)
					s += (pts[k].ToVec3f() - pts[k - 1].ToVec3f()).length();
				return s;
			};

		std::vector<bool> lakeVisited(pureLakes.size(), false);
		for (size_t lakeStep = 0; lakeStep < pureLakes.size(); lakeStep++)
		{
			size_t lIdx = 0;
			if (waypoints.empty())
			{
				while (lIdx < pureLakes.size() && lakeVisited[lIdx]) lIdx++;
			}
			else
			{
				const Vec3f refPos = waypoints.back().ToVec3f();
				float bestLakeDistSq = std::numeric_limits<float>::max();
				for (size_t cand = 0; cand < pureLakes.size(); cand++)
				{
					if (lakeVisited[cand]) continue;
					const float dSq = std::min(
						(pureLakes[cand].arcA.front().ToVec3f() - refPos).length_sqr(),
						(pureLakes[cand].arcA.back().ToVec3f() - refPos).length_sqr());
					if (dSq < bestLakeDistSq) { bestLakeDistSq = dSq; lIdx = cand; }
				}
			}
			lakeVisited[lIdx] = true;

			std::vector<float3> arcA = pureLakes[lIdx].arcA;
			std::vector<float3> arcB = pureLakes[lIdx].arcB;

			if (!waypoints.empty())
			{
				const Vec3f refPos = waypoints.back().ToVec3f();
				if ((arcA.back().ToVec3f() - refPos).length_sqr() < (arcA.front().ToVec3f() - refPos).length_sqr())
				{
					std::reverse(arcA.begin(), arcA.end());
					std::reverse(arcB.begin(), arcB.end());
				}
			}

			constexpr int cSamplesAlongLake = 80;
			const auto sampA = resampleArcUniform(arcA, cSamplesAlongLake);
			const auto sampB = resampleArcUniform(arcB, cSamplesAlongLake);

			const float avgArcLen = std::max(1.0f, 0.5f * (polylineLen(arcA) + polylineLen(arcB)));
			const float maxLakeWidthMm = std::max(1.0f, 1.5f * pureLakes[lIdx].areaMm2 / avgArcLen);
			const float targetCrossStep = std::clamp(flatStep, minStepMm, maxStepMm);
			const int numCrossPasses = std::clamp(static_cast<int>(std::ceil(maxLakeWidthMm / targetCrossStep)), 2, 12);

			transitionSafeClPoint(arcA.front());

			for (int p = 0; p <= numCrossPasses; p++)
			{
				const float tau = static_cast<float>(p) / numCrossPasses;
				const bool forwardS = ((p % 2) == 0);
				std::vector<float3> passPts;
				passPts.reserve(cSamplesAlongLake + 1);
				for (int idx = 0; idx <= cSamplesAlongLake; idx++)
				{
					const int s = forwardS ? idx : (cSamplesAlongLake - idx);
					const Vec3f pt = Vec3f::lerp(sampA[s], sampB[s], tau);
					passPts.emplace_back(pt.x, minSafeTipY, pt.z);
				}
				for (const auto& sp : SimplifyPolyline3DGougeSafe(passPts, simplifyTol))
					appendSafePoint(sp);
			}

		}
	}

	}

	void AppendPencilMillingPasses(
		const std::vector<TracedFinishingIntersection>& tracedIntersections,
		const std::vector<std::pair<SceneObject*, bool>>& orderedSurfaces,
		const OffsetSurfaceMap& offsetSurfaces,
		const std::unordered_map<SceneObject*, size_t>& objToSurfIdx,
		const std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>>& surfAttachments,
		float R,
		float baseY,
		float minSafeTipY,
		float yCenterMinWorld,
		float halfX,
		float halfZ,
		float safeY,
		float simplifyTol,
		FinishingPathWriter& writer)
	{
		auto& waypoints = writer.waypoints;
		auto appendSafePoint = [&](const float3& pt) { writer.AppendSafePoint(pt); };
		auto transitionSafeCl = [&](const std::vector<float3>& pts) { EmitAheadTransition(writer, pts); };
		constexpr float cMinSafeHorizontalSlope = FinishingPathWriter::cMinSafeHorizontalSlope;

	// 8. Pencil milling passes along exact offset-surface intersections in greedy nearest-neighbor order
	{
		std::vector<std::vector<float3>> activeGrooveChains = BuildPencilMillingChains(
			tracedIntersections, orderedSurfaces, offsetSurfaces, objToSurfIdx, surfAttachments,
			yCenterMinWorld, R, halfX, halfZ, baseY, minSafeTipY, simplifyTol);

		std::vector<bool> grooveUsed(activeGrooveChains.size(), false);
		for (size_t step = 0; step < activeGrooveChains.size(); step++)
		{
			Vec3f refPos = waypoints.empty() ? Vec3f(0.0f, safeY, 0.0f) : waypoints.back().ToVec3f();
			float bestCost = std::numeric_limits<float>::max();
			std::optional<size_t> bestIdx;
			bool reverseBest = false;

			for (size_t cIdx = 0; cIdx < activeGrooveChains.size(); cIdx++)
			{
				if (grooveUsed[cIdx] || activeGrooveChains[cIdx].size() < 2)
					continue;
				const auto& ch = activeGrooveChains[cIdx];
				for (int side = 0; side < 2; side++)
				{
					const Vec3f cand = (side == 0) ? ch.front().ToVec3f() : ch.back().ToVec3f();
					const float dist = (cand - refPos).length();
					const bool directShort = !waypoints.empty() && dist <= 2.0f;
					const float cost = directShort ? dist : (dist + 40.0f);

					if (cost < bestCost)
					{
						bestCost = cost;
						bestIdx = cIdx;
						reverseBest = (side == 1);
					}
				}
			}

			if (!bestIdx)
				break;

			grooveUsed[*bestIdx] = true;
			std::vector<float3> ch = activeGrooveChains[*bestIdx];
			if (reverseBest)
				std::reverse(ch.begin(), ch.end());

			{
				const Vec3f p0 = ch.front().ToVec3f();
				const Vec3f p1 = ch.back().ToVec3f();
				const float dy = p1.y - p0.y;
				const float lenXZ = std::hypot(p1.x - p0.x, p1.z - p0.z);
				if (dy < -0.5f && lenXZ < (-dy) * cMinSafeHorizontalSlope * 1.25f)
					std::reverse(ch.begin(), ch.end());
			}

			const Vec3f startPt = ch.front().ToVec3f();
			const float grooveGap = waypoints.empty() ? 1e9f : (startPt - waypoints.back().ToVec3f()).length();
			if (!waypoints.empty() && grooveGap <= 0.15f)
			{
				// Already at startPt
			}
			else if (!waypoints.empty() && grooveGap <= 2.0f)
			{
				appendSafePoint(ch.front());
			}

			else
			{
				transitionSafeCl(ch);
			}

			for (size_t k = 1; k < ch.size(); k++)
				appendSafePoint(ch[k]);
		}
	}

	}
}

std::vector<float3> PathGenerator::GenerateFinishingPath(
	const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
	const FinishingParameters& params)
{

	const float R = params.cutterDiameter * 0.5f;
	const float offsetWorld = R * cMmToWorld;
	const float stepWorld = 0.5f * cMmToWorld;
	const float scallopH = std::clamp(params.scallopHeight, 0.002f, 0.5f);
	const float minStepMm = std::clamp(params.minStepover, 0.10f, 5.0f);
	const float maxStepMm = std::clamp(params.maxStepover, minStepMm, 6.0f);
	const float curvWeight = std::max(0.0f, params.curvatureWeight);
	const float baseFlatWidth = 2.0f * std::sqrt(2.0f * R * scallopH);
	const float simplifyTol = std::max(0.005f, params.simplifyTolerance);
	const float safeY = std::max(params.safeHeight, params.blockSizeY + 10.0f);
	const float halfX = params.blockSizeX * 0.5f;
	const float halfZ = params.blockSizeZ * 0.5f;
	constexpr float cOutsideBlockMarginMm = 2.0f;

	float baseY = params.baseHeight;
	if (params.baseSurface != nullptr)
	{
		auto eval = GeometryMath::EvaluateParametric(params.baseSurface, 0.5f, 0.5f);
		baseY = std::max(baseY, eval.p.y * cWorldToMm);
	}
	const float minSafeTipY = baseY + 0.015f;
	const float yCenterMinWorld = (baseY + R) * cMmToWorld;

	FinishingConfig cfg = LoadFinishingConfigFromJson(params.intersectionsJsonPath);

	auto findObjectById = [&](uint32_t id) -> SceneObject*
		{
			auto it = std::find_if(sceneObjects.begin(), sceneObjects.end(),
				[id](const auto& obj) { return obj && obj->m_id == id; });
			return (it != sceneObjects.end()) ? it->get() : nullptr;
		};

	// 1. Resolve ordered surfaces and their leading parametric directions
	std::vector<std::pair<SceneObject*, bool>> orderedSurfaces;
	for (const auto& spec : cfg.surfaces)
	{
		if (SceneObject* obj = findObjectById(spec.id))
			orderedSurfaces.emplace_back(obj, spec.alongV);
	}

	if (orderedSurfaces.empty())
		return {};

	const size_t numSurf = orderedSurfaces.size();
	std::unordered_map<SceneObject*, size_t> objToSurfIdx;
	OffsetSurfaceMap offsetSurfaces;
	auto getOrCreateOffset = [&](SceneObject* obj) -> const std::shared_ptr<GeometryMath::OffsetSurface>&
		{
			auto& ptr = offsetSurfaces[obj];
			if (!ptr)
				ptr = std::make_shared<GeometryMath::OffsetSurface>(obj, offsetWorld, false);
			return ptr;
		};
	for (size_t i = 0; i < numSurf; i++)
	{
		objToSurfIdx[orderedSurfaces[i].first] = i;
		getOrCreateOffset(orderedSurfaces[i].first);
	}

	// 2. Compute exact offset-surface intersections (CL intersections) from JSON seeds in parallel
	struct PairSeedJob
	{
		SceneObject* objA = nullptr;
		SceneObject* objB = nullptr;
		std::optional<float3> seed;
	};
	std::vector<PairSeedJob> pairJobs;
	for (const auto& ispec : cfg.intersections)
	{
		SceneObject* objA = findObjectById(ispec.idA);
		SceneObject* objB = findObjectById(ispec.idB);
		if (!objA || !objB)
			continue;

		getOrCreateOffset(objA);
		getOrCreateOffset(objB);
		if (ispec.cursorSeeds.empty())
		{
			pairJobs.push_back({ objA, objB, std::nullopt });
		}
		else
		{
			for (const auto& seed : ispec.cursorSeeds)
				pairJobs.push_back({ objA, objB, seed });
		}
	}

	std::vector<std::optional<TracedFinishingIntersection>> jobResults(pairJobs.size());
#pragma omp parallel for schedule(dynamic, 1)
	for (int idx = 0; idx < static_cast<int>(pairJobs.size()); idx++)
	{
		const auto& job = pairJobs[idx];
		const auto& clA = *offsetSurfaces.at(job.objA);
		const auto& clB = *offsetSurfaces.at(job.objB);
		Vec4f maxDomains(clA.GetMaxU(), clA.GetMaxV(), clB.GetMaxU(), clB.GetMaxV());
		bool isSelf = (job.objA == job.objB);

		auto resOpt = GeometryMath::CalculateIntersection(clA, clB, isSelf, maxDomains, stepWorld, job.seed, true);

		if (resOpt && resOpt->points.size() >= 2)
		{
			jobResults[idx] = TracedFinishingIntersection{
				job.objA,
				job.objB,
				std::move(resOpt->points),
				std::move(resOpt->params),
				resOpt->isClosedLoop
			};
		}
	}

	std::vector<TracedFinishingIntersection> tracedIntersections;
	tracedIntersections.reserve(pairJobs.size());
	for (auto& jr : jobResults)
	{
		if (jr)
			tracedIntersections.push_back(std::move(*jr));
	}

	// 3. Build per-surface unwrapped UV curve attachments for trimming (surface-surface + surface-base)
	std::unordered_map<SceneObject*, std::vector<SurfaceCurveAttachment>> surfAttachments;
	for (const auto& traced : tracedIntersections)
	{
		const auto& clA = offsetSurfaces.at(traced.objA);
		const auto& clB = offsetSurfaces.at(traced.objB);

		std::vector<Vec2f> uvA(traced.params.size());
		std::vector<Vec2f> uvB(traced.params.size());
		for (size_t k = 0; k < traced.params.size(); k++)
		{
			uvA[k] = { traced.params[k].x, traced.params[k].y };
			uvB[k] = { traced.params[k].z, traced.params[k].w };
		}

		surfAttachments[traced.objA].push_back({
			traced.objB,
			clB,
			BuildUnwrappedUVSegments(*clA, *clB, traced.params, true),
			uvA,
			uvB,
			traced.pointsWorld,
			traced.isClosedLoop
			});

		if (traced.objB != traced.objA)
		{
			surfAttachments[traced.objB].push_back({
				traced.objA,
				clA,
				BuildUnwrappedUVSegments(*clB, *clA, traced.params, false),
				std::move(uvB),
				std::move(uvA),
				traced.pointsWorld,
				traced.isClosedLoop
				});
		}
	}

	const float halfXWorld = halfX * cMmToWorld;
	const float halfZWorld = halfZ * cMmToWorld;
	const float planePadWorld = 100.0f * cMmToWorld;
	auto horizPlane = std::make_shared<GeometryMath::HorizontalPlaneSurface>(
		yCenterMinWorld,
		-halfXWorld - planePadWorld, halfXWorld + planePadWorld,
		-halfZWorld - planePadWorld, halfZWorld + planePadWorld);

	auto baseSeedsById = LoadBaseCursorSeedsFromJson(params.intersectionsJsonPath);
	std::vector<std::vector<SurfaceCurveAttachment>> perSurfBaseAttachments(numSurf);
#pragma omp parallel for schedule(dynamic, 1)
	for (int sIdx = 0; sIdx < static_cast<int>(numSurf); sIdx++)
	{
		SceneObject* obj = orderedSurfaces[sIdx].first;
		const auto& clSelf = *offsetSurfaces.at(obj);
		CollectBasePlaneIntersections(obj, clSelf, horizPlane, stepWorld, baseSeedsById, perSurfBaseAttachments[sIdx]);
	}
	for (size_t sIdx = 0; sIdx < numSurf; sIdx++)
	{
		SceneObject* obj = orderedSurfaces[sIdx].first;
		auto& dst = surfAttachments[obj];
		auto& src = perSurfBaseAttachments[sIdx];
		dst.insert(dst.end(), std::make_move_iterator(src.begin()), std::make_move_iterator(src.end()));
	}

	// 4. Lightweight 250x250 clearance map for low-altitude corridor transfers
	const FinishingClearanceMap clearanceMap = BuildFinishingClearanceMap(
		sceneObjects, params, R, baseY, minSafeTipY);
	const float globalModelClearY = std::min(safeY, std::max(minSafeTipY + 5.0f, clearanceMap.maxModelSurfY + 4.0f));
	// 5. Path writer encapsulating slope-safe insertion and corridor-aware transfers
	FinishingPathWriter writer{
		{}, 0, 0.0,
		halfX, halfZ, R, baseY, minSafeTipY, safeY, globalModelClearY,
		clearanceMap
	};
	writer.waypoints.reserve(16384);

	auto& waypoints = writer.waypoints;
	auto& totalAirMoveMm = writer.totalAirMoveMm;

	// 6. Zigzag finishing passes on each surface in helper JSON order
	AppendSurfaceZigzagPasses(
		orderedSurfaces, offsetSurfaces, surfAttachments, params,
		R, baseY, minSafeTipY, yCenterMinWorld, halfX, halfZ, safeY, cOutsideBlockMarginMm, writer);

	// 7. Clear enclosed flat base lakes purely from trimmed OffsetSurface x HorizontalPlaneSurface contours
	AppendEnclosedLakePasses(
		orderedSurfaces, offsetSurfaces, surfAttachments, params,
		R, minSafeTipY, yCenterMinWorld, halfX, halfZ, writer);

	// 8. Pencil milling passes along exact offset-surface intersections in greedy nearest-neighbor order
	AppendPencilMillingPasses(
		tracedIntersections, orderedSurfaces, offsetSurfaces, objToSurfIdx, surfAttachments,
		R, baseY, minSafeTipY, yCenterMinWorld, halfX, halfZ, safeY, simplifyTol, writer);

	// 9. Final retract to safe height
	const float3& lastPt = waypoints.back();
	if (lastPt.y < safeY - 1e-3f)
	{
		totalAirMoveMm += (safeY - lastPt.y);
		waypoints.emplace_back(lastPt.x, safeY, lastPt.z);
	}

	return waypoints;
}

bool PathGenerator::ExportGCode(
	const std::string& filepath,
	const std::vector<float3>& sceneMmWaypoints)
{
	if (sceneMmWaypoints.empty())
		return false;

	std::ofstream file(filepath, std::ios::out | std::ios::trunc);
	if (!file.is_open())
		return false;

	file << std::fixed << std::setprecision(3);

	double totalLengthMm = 0.0;
	Vec3f prevPt = sceneMmWaypoints.front().ToVec3f();
	size_t lineNum = 3;
	for (const auto& scenePt : sceneMmWaypoints)
	{
		Vec3f currPt = scenePt.ToVec3f();
		totalLengthMm += (currPt - prevPt).length();
		prevPt = currPt;

		float3 gcodePt = SceneToGCodeCoords(scenePt);
		file << "N" << lineNum++
			<< "G01"
			<< "X" << gcodePt.x
			<< "Y" << gcodePt.y
			<< "Z" << gcodePt.z
			<< "\n";
	}

	if (!file.good())
		return false;

	// Log export statistics (points and total path length) in append mode
	std::filesystem::path exportPath(filepath);
	std::filesystem::path logPath = exportPath.has_parent_path()
		? exportPath.parent_path() / "export_log.txt"
		: std::filesystem::path("export_log.txt");


	std::ofstream logFile(logPath, std::ios::out | std::ios::app);
	if (logFile.is_open())
	{
		logFile << std::fixed << std::setprecision(3);
		logFile << "File: " << exportPath.filename().string()
			<< " | Points: " << sceneMmWaypoints.size()
			<< " | Total length: " << totalLengthMm << " mm\n";
	}

	return true;
}
