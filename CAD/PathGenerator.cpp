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
