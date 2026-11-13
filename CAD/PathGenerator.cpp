#include "pch.h"
#include "PathGenerator.h"
#include "GeometryMath.h"
#include "Surface.h"
#include "Torus.h"
#include <omp.h>
#include <atomic>
#include <cmath>

using namespace MathLib;

namespace
{
	constexpr float cMmToWorld = 0.05f;
	constexpr float cWorldToMm = 1.0f / cMmToWorld; // 20.0f

	struct KernelOffset
	{
		int ox;
		int oz;
		float sphereHeightOffset; // -R + sqrt(R^2 - d^2)
	};

	struct SegmentTask
	{
		SceneObject* obj;
		float u0, u1;
		float v0, v1;
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

		int subU = std::clamp(static_cast<int>(std::ceil(maxEdgeU / targetStepMm)), 1, 16);
		int subV = std::clamp(static_cast<int>(std::ceil(maxEdgeV / targetStepMm)), 1, 16);

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

	void SampleSegmentToHeightMap(
		SceneObject* obj,
		float uStart,
		float uEnd,
		float vStart,
		float vEnd,
		float halfX,
		float halfZ,
		float dx,
		float dz,
		int divX,
		int divZ,
		std::vector<float>& heightGrid)
	{
		const float invDx = 1.0f / dx;
		const float invDz = 1.0f / dz;
		const float targetStepMm = 0.4f * std::min(dx, dz); // <= half grid cell size

		// Coarse probe across the segment to bound maximum derivatives
		float segMaxDuMm = 1.0f;
		float segMaxDvMm = 1.0f;
		constexpr int cProbeSamples = 6;
		for (int iv = 0; iv <= cProbeSamples; iv++)
		{
			float v = std::lerp(vStart, vEnd, static_cast<float>(iv) / cProbeSamples);
			for (int iu = 0; iu <= cProbeSamples; iu++)
			{
				float u = std::lerp(uStart, uEnd, static_cast<float>(iu) / cProbeSamples);
				auto eval = GeometryMath::EvaluateParametric(obj, u, v);
				segMaxDuMm = std::max(segMaxDuMm, eval.du.length() * cWorldToMm);
				segMaxDvMm = std::max(segMaxDvMm, eval.dv.length() * cWorldToMm);
			}
		}

		// Adaptive step counts for this segment based on derivative magnitude
		const int stepsU = std::clamp(static_cast<int>(std::ceil((uEnd - uStart) * segMaxDuMm * 1.25f / targetStepMm)), 64, 1200);
		const int stepsV = std::clamp(static_cast<int>(std::ceil((vEnd - vStart) * segMaxDvMm * 1.25f / targetStepMm)), 64, 1200);

		std::vector<Vec3f> prevRow(stepsU + 1);
		std::vector<Vec3f> currRow(stepsU + 1);

		for (int iv = 0; iv <= stepsV; iv++)
		{
			float v = std::lerp(vStart, vEnd, static_cast<float>(iv) / stepsV);
			for (int iu = 0; iu <= stepsU; iu++)
			{
				float u = std::lerp(uStart, uEnd, static_cast<float>(iu) / stepsU);
				auto eval = GeometryMath::EvaluateParametric(obj, u, v);
				Vec3f posMm = eval.p * cWorldToMm;
				currRow[iu] = posMm;
				SplatPointToGrid(posMm, halfX, halfZ, invDx, invDz, divX, divZ, heightGrid);

				if (iv > 0 && iu > 0)
				{
					SplatQuadSubdivided(
						prevRow[iu - 1], prevRow[iu],
						currRow[iu - 1], currRow[iu],
						targetStepMm, halfX, halfZ, invDx, invDz, divX, divZ, heightGrid);
				}
			}
			std::swap(prevRow, currRow);
		}
	}

	std::vector<float> BuildSurfaceHeightMap(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const RoughingParameters& params)
	{
		const int cols = params.gridDivX + 1;
		const int rows = params.gridDivZ + 1;
		const float halfX = params.blockSizeX * 0.5f;
		const float halfZ = params.blockSizeZ * 0.5f;
		const float dx = params.blockSizeX / params.gridDivX;
		const float dz = params.blockSizeZ / params.gridDivZ;

		std::vector<float> heightGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());

		std::vector<SegmentTask> tasks;
		tasks.reserve(350);

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

		#pragma omp parallel for schedule(dynamic, 1)
		for (int i = 0; i < static_cast<int>(tasks.size()); i++)
		{
			const auto& task = tasks[i];
			SampleSegmentToHeightMap(
				task.obj,
				task.u0, task.u1,
				task.v0, task.v1,
				halfX, halfZ, dx, dz,
				params.gridDivX, params.gridDivZ,
				heightGrid);
		}

		return heightGrid;
	}

	std::vector<float> BuildCutterLocationHeightMap(
		const std::vector<float>& surfaceHeightGrid,
		const RoughingParameters& params)
	{
		const int divX = params.gridDivX;
		const int divZ = params.gridDivZ;
		const int cols = divX + 1;
		const int rows = divZ + 1;
		const float dx = params.blockSizeX / divX;
		const float dz = params.blockSizeZ / divZ;

		const float R = params.cutterDiameter * 0.5f;
		// Inflate effective check radius slightly by half grid diagonal to protect sub-cell geometry
		const float cellHalfDiag = 0.5f * std::sqrt(dx * dx + dz * dz);
		const float effectiveR = R + cellHalfDiag;
		const float effectiveR2 = effectiveR * effectiveR;

		const int radCellsX = static_cast<int>(std::ceil(effectiveR / dx));
		const int radCellsZ = static_cast<int>(std::ceil(effectiveR / dz));

		std::vector<KernelOffset> kernel;
		kernel.reserve((static_cast<size_t>(2) * radCellsX + 1) * (static_cast<size_t>(2) * radCellsZ + 1));

		for (int oz = -radCellsZ; oz <= radCellsZ; oz++)
		{
			float distZ = oz * dz;
			for (int ox = -radCellsX; ox <= radCellsX; ox++)
			{
				float distX = ox * dx;
				float distSq = distX * distX + distZ * distZ;
				if (distSq <= effectiveR2)
				{
					// Tip of ball-nose cutter is at y_tip; sphere surface at distance d is y_tip + R - sqrt(R^2 - d^2).
					// To keep sphere >= H, we need y_tip >= H - R + sqrt(R^2 - d^2).
					float sphereOffset = -R + std::sqrt(std::max(0.0f, R * R - distSq));
					kernel.push_back({ ox, oz, sphereOffset });
				}
			}
		}

		const float minSafeBaseHeight = params.baseHeight + params.baseMargin;
		std::vector<float> clGrid(static_cast<size_t>(cols) * rows, minSafeBaseHeight);

		#pragma omp parallel for schedule(dynamic, 8)
		for (int jz = 0; jz <= divZ; jz++)
		{
			size_t srcRowOffset = static_cast<size_t>(jz) * cols;
			for (int jx = 0; jx <= divX; jx++)
			{
				float rawH = surfaceHeightGrid[srcRowOffset + jx];
				if (rawH <= -1e5f)
					continue;

				float paddedH = rawH + params.surfaceMargin;
				if (paddedH <= minSafeBaseHeight)
					continue;

				for (const auto& k : kernel)
				{
					int ix = jx + k.ox;
					int iz = jz + k.oz;
					if (ix >= 0 && ix <= divX && iz >= 0 && iz <= divZ)
					{
						float requiredTipY = paddedH + k.sphereHeightOffset;
						size_t dstIdx = static_cast<size_t>(iz) * cols + ix;
						if (requiredTipY > clGrid[dstIdx])
						{
							AtomicMaxFloat(&clGrid[dstIdx], requiredTipY);
						}
					}
				}
			}
		}

		// Apply downward slope limiter so no steep cliff on Z_CL can cause a near-vertical plunge
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

	// 1. Build dense surface height map from all non-base surfaces & toruses
	std::vector<float> surfaceHeightGrid = BuildSurfaceHeightMap(sceneObjects, effectiveParams);

	// 2. Compute ball-nose cutter location (CL) height map to prevent undercutting V-valleys & steep walls
	std::vector<float> clGrid = BuildCutterLocationHeightMap(surfaceHeightGrid, effectiveParams);

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
	if (!waypoints.empty())
	{
		const float3& lastPt = waypoints.back();
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

	size_t lineNum = 3;
	for (const auto& scenePt : sceneMmWaypoints)
	{
		float3 gcodePt = SceneToGCodeCoords(scenePt);
		file << "N" << lineNum++
			<< "G01"
			<< "X" << gcodePt.x
			<< "Y" << gcodePt.y
			<< "Z" << gcodePt.z
			<< "\n";
	}

	return file.good();
}
