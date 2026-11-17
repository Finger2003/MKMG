#include "pch.h"
#include "PathGenerator.h"
#include "GeometryMath.h"
#include "Surface.h"
#include "Torus.h"
#include <omp.h>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

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
