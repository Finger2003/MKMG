#include "pch.h"
#include "PathGenerator.h"
#include "GeometryMath.h"
#include "Surface.h"
#include "Torus.h"
#include "../MathLib/Vec2f.h"
#include <omp.h>
#include <atomic>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <utility>
#include <iomanip>
#include <filesystem>
#include <nlohmann/json.hpp>

using namespace MathLib;

namespace
{
	constexpr float cMmToWorld = 0.05f;
	constexpr float cWorldToMm = 1.0f / cMmToWorld; // 20.0f
	constexpr float cOutsideBlockMarginMm = 2.0f;

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

	struct Pt2D
	{
		float x = 0.0f;
		float z = 0.0f;

		Pt2D operator+(const Pt2D& rhs) const { return { x + rhs.x, z + rhs.z }; }
		Pt2D operator-(const Pt2D& rhs) const { return { x - rhs.x, z - rhs.z }; }
		Pt2D operator*(float s) const { return { x * s, z * s }; }
		Pt2D operator/(float s) const { return operator*(1.0f / s); }

		float lengthSq() const { return x * x + z * z; }
		float length() const { return std::sqrt(lengthSq()); }

		Pt2D normalized() const
		{
			float l = length();
			return (l > 1e-6f) ? (*this * (1.0f / l)) : Pt2D{ 0.0f, 0.0f };
		}

		Pt2D perp() const { return { z, -x }; }

		static float dot(const Pt2D& a, const Pt2D& b)
		{
			return a.x * b.x + a.z * b.z;
		}

		static float cross(const Pt2D& a, const Pt2D& b)
		{
			return a.x * b.z - a.z * b.x;
		}

		static Pt2D lerp(const Pt2D& a, const Pt2D& b, float t)
		{
			return { std::lerp(a.x, b.x, t), std::lerp(a.z, b.z, t) };
		}
	};

	struct Poly2D
	{
		std::vector<Pt2D> pts;
		float minX = 0.0f;
		float maxX = 0.0f;
		float minZ = 0.0f;
		float maxZ = 0.0f;
	};

	Poly2D BuildPolygon2DFromWorldPoints(const std::vector<Vec3f>& worldPts)
	{
		Poly2D poly;
		if (worldPts.size() < 3)
			return poly;

		poly.pts.reserve(worldPts.size());
		for (const auto& wp : worldPts)
		{
			Pt2D pt{ wp.x * cWorldToMm, wp.z * cWorldToMm };
			if (poly.pts.empty() || (pt - poly.pts.back()).lengthSq() > 1e-8f)
				poly.pts.push_back(pt);
		}

		if (poly.pts.size() >= 2 && (poly.pts.front() - poly.pts.back()).lengthSq() < 1e-6f)
			poly.pts.pop_back();

		if (poly.pts.size() < 3)
		{
			poly.pts.clear();
			return poly;
		}

		float area2 = 0.0f;
		const size_t n = poly.pts.size();
		for (size_t i = 0; i < n; i++)
		{
			const Pt2D& a = poly.pts[i];
			const Pt2D& b = poly.pts[(i + 1) % n];
			area2 += Pt2D::cross(a, b);
		}
		if (area2 < 0.0f)
			std::reverse(poly.pts.begin(), poly.pts.end());

		poly.minX = poly.maxX = poly.pts.front().x;
		poly.minZ = poly.maxZ = poly.pts.front().z;
		for (const auto& pt : poly.pts)
		{
			poly.minX = std::min(poly.minX, pt.x);
			poly.maxX = std::max(poly.maxX, pt.x);
			poly.minZ = std::min(poly.minZ, pt.z);
			poly.maxZ = std::max(poly.maxZ, pt.z);
		}

		return poly;
	}

	std::vector<Pt2D> BuildOutwardOffsetLoop2D(const Poly2D& poly, float R, float maxSubStep = 0.15f)
	{
		const size_t n = poly.pts.size();
		if (n < 3)
			return {};

		std::vector<Pt2D> rawOff;
		rawOff.reserve(n * 2);

		for (size_t i = 0; i < n; i++)
		{
			const Pt2D& pPrev = poly.pts[(i + n - 1) % n];
			const Pt2D& pCurr = poly.pts[i];
			const Pt2D& pNext = poly.pts[(i + 1) % n];

			Pt2D e1 = pCurr - pPrev;
			Pt2D e2 = pNext - pCurr;
			float l1 = e1.length();
			float l2 = e2.length();
			if (l1 < 1e-6f || l2 < 1e-6f)
				continue;

			Pt2D t1 = e1 / l1;
			Pt2D t2 = e2 / l2;
			Pt2D n1 = t1.perp();
			Pt2D n2 = t2.perp();

			float cross = Pt2D::cross(t1, t2);
			float dot = std::clamp(Pt2D::dot(n1, n2), -1.0f, 1.0f);
			float angle = std::acos(dot);

			if (cross > 0.0f && angle > 0.02f)
			{
				int steps = std::max(2, static_cast<int>(std::ceil(angle / 0.02f)));
				float a1 = std::atan2(n1.z, n1.x);
				float a2 = std::atan2(n2.z, n2.x);
				float da = std::remainder(a2 - a1, 2.0f * std::numbers::pi_v<float>);
				for (int s = 0; s <= steps; s++)
				{
					float a = a1 + da * (static_cast<float>(s) / steps);
					rawOff.push_back({ pCurr.x + R * std::cos(a), pCurr.z + R * std::sin(a) });
				}
			}
			else
			{
				Pt2D avgN = n1 + n2;
				float nl = avgN.length();
				avgN = (nl > 1e-6f) ? (avgN * (1.0f / nl)) : n1;
				rawOff.push_back(pCurr + avgN * R);
			}
		}

		std::vector<Pt2D> denseOff;
		const size_t m = rawOff.size();
		denseOff.reserve(m * 2);
		for (size_t i = 0; i < m; i++)
		{
			const Pt2D& p0 = rawOff[i];
			const Pt2D& p1 = rawOff[(i + 1) % m];
			float dist = (p1 - p0).length();
			int sub = std::max(1, static_cast<int>(std::ceil(dist / maxSubStep)));
			for (int s = 0; s < sub; s++)
			{
				float t = static_cast<float>(s) / sub;
				denseOff.push_back(Pt2D::lerp(p0, p1, t));
			}
		}

		return denseOff;
	}

	bool IsPointInPolygon2D(const Pt2D& pt, const Poly2D& poly)
	{
		if (pt.x < poly.minX || pt.x > poly.maxX || pt.z < poly.minZ || pt.z > poly.maxZ)
			return false;

		bool inside = false;
		const size_t n = poly.pts.size();
		for (size_t i = 0; i < n; i++)
		{
			const Pt2D& a = poly.pts[i];
			const Pt2D& b = poly.pts[(i + 1) % n];
			if ((a.z > pt.z) != (b.z > pt.z))
			{
				if ((b.z > a.z) == (Pt2D::cross(b - a, pt - a) > 0.0f))
					inside = !inside;
			}
		}
		return inside;
	}

	float PointToSegmentDistSq2D(const Pt2D& p, const Pt2D& a, const Pt2D& b)
	{
		Pt2D ab = b - a;
		float lenSq = ab.lengthSq();
		if (lenSq < 1e-12f)
			return (p - a).lengthSq();

		float t = std::clamp(Pt2D::dot(p - a, ab) / lenSq, 0.0f, 1.0f);
		Pt2D proj = a + ab * t;
		return (p - proj).lengthSq();
	}

	bool IsDistToPolygonAtLeast2D(const Pt2D& pt, const Poly2D& poly, float minDist)
	{
		float dx = std::max({ 0.0f, poly.minX - pt.x, pt.x - poly.maxX });
		float dz = std::max({ 0.0f, poly.minZ - pt.z, pt.z - poly.maxZ });
		float minDistSq = minDist * minDist;
		if (dx * dx + dz * dz >= minDistSq)
			return true;

		const size_t n = poly.pts.size();
		for (size_t i = 0; i < n; i++)
		{
			if (PointToSegmentDistSq2D(pt, poly.pts[i], poly.pts[(i + 1) % n]) < minDistSq)
				return false;
		}
		return true;
	}

	std::vector<Pt2D> SimplifyPolyline2D(const std::vector<Pt2D>& rawPts, float tolerance)
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

			const Pt2D& pStart = rawPts[iStart];
			const Pt2D& pEnd = rawPts[iEnd];

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

		std::vector<Pt2D> simplified;
		simplified.reserve(n);
		for (int i = 0; i < n; i++)
		{
			if (keep[i])
				simplified.push_back(rawPts[i]);
		}
		return simplified;
	}

	std::optional<Pt2D> SegmentIntersection2D(const Pt2D& a0, const Pt2D& a1, const Pt2D& b0, const Pt2D& b1)
	{
		Pt2D r = a1 - a0;
		Pt2D s = b1 - b0;
		float denom = Pt2D::cross(r, s);
		if (std::abs(denom) < 1e-8f)
			return std::nullopt;

		Pt2D qmp = b0 - a0;
		float t = Pt2D::cross(qmp, s) / denom;
		float u = Pt2D::cross(qmp, r) / denom;
		if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f)
		{
			Pt2D ptA = a0 + r * t;
			Pt2D ptB = b0 + s * u;
			return (ptA + ptB) * 0.5f;
		}

		return std::nullopt;
	}

	void AppendChainTrimmingOverlap(std::vector<Pt2D>& contour, const std::vector<Pt2D>& nextChain)
	{
		if (nextChain.empty())
			return;
		if (contour.size() < 2 || nextChain.size() < 2)
		{
			contour.insert(contour.end(), nextChain.begin(), nextChain.end());
			return;
		}

		constexpr size_t cLookback = 12;
		size_t tailStart = (contour.size() > cLookback) ? (contour.size() - cLookback) : 0;
		size_t headEnd = std::min(cLookback, nextChain.size() - 1);

		for (size_t i = contour.size() - 1; i > tailStart; i--)
		{
			for (size_t j = 0; j < headEnd; j++)
			{
				if (auto hit = SegmentIntersection2D(contour[i - 1], contour[i], nextChain[j], nextChain[j + 1]))
				{
					contour.resize(i);
					if ((contour.back() - *hit).lengthSq() > 1e-6f)
						contour.push_back(*hit);
					contour.insert(contour.end(), nextChain.begin() + j + 1, nextChain.end());
					return;
				}
			}
		}

		contour.insert(contour.end(), nextChain.begin(), nextChain.end());
	}

	std::vector<Pt2D> BuildOuterReachableContour(
		const std::vector<Poly2D>& intersectionPolys,
		float offsetRadius,
		float validTolerance = 0.015f)
	{
		if (intersectionPolys.empty())
			return {};

		std::vector<std::vector<Pt2D>> offsetLoops(intersectionPolys.size());
		for (size_t i = 0; i < intersectionPolys.size(); i++)
			offsetLoops[i] = BuildOutwardOffsetLoop2D(intersectionPolys[i], offsetRadius, 0.15f);

		std::vector<std::vector<Pt2D>> validChains;
		const float minSafeDist = std::max(0.0f, offsetRadius - validTolerance);

		for (const auto& off : offsetLoops)
		{
			const size_t n = off.size();
			if (n < 2)
				continue;

			std::vector<uint8_t> valid(n, 1);
			#pragma omp parallel for schedule(static)
			for (int i = 0; i < static_cast<int>(n); i++)
			{
				const Pt2D& pt = off[i];
				for (const auto& poly : intersectionPolys)
				{
					if (IsPointInPolygon2D(pt, poly) || !IsDistToPolygonAtLeast2D(pt, poly, minSafeDist))
					{
						valid[i] = 0;
						break;
					}
				}
			}

			size_t firstFalse = n;
			for (size_t i = 0; i < n; i++)
			{
				if (!valid[i])
				{
					firstFalse = i;
					break;
				}
			}

			if (firstFalse == n)
			{
				validChains.push_back(off);
				continue;
			}

			std::vector<Pt2D> currentRun;
			for (size_t k = 0; k < n; k++)
			{
				size_t idx = (firstFalse + 1 + k) % n;
				if (valid[idx])
				{
					currentRun.push_back(off[idx]);
				}
				else if (!currentRun.empty())
				{
					if (currentRun.size() >= 2)
						validChains.push_back(std::move(currentRun));
					currentRun.clear();
				}
			}
			if (currentRun.size() >= 2)
				validChains.push_back(std::move(currentRun));
		}

		if (validChains.empty())
			return {};

		size_t startChainIdx = 0;
		float globalMinZ = std::numeric_limits<float>::max();
		for (size_t i = 0; i < validChains.size(); i++)
		{
			for (const auto& pt : validChains[i])
			{
				if (pt.z < globalMinZ)
				{
					globalMinZ = pt.z;
					startChainIdx = i;
				}
			}
		}

		std::vector<bool> used(validChains.size(), false);
		std::vector<size_t> orderedIndices;
		orderedIndices.push_back(startChainIdx);
		used[startChainIdx] = true;

		while (true)
		{
			const Pt2D& currEnd = validChains[orderedIndices.back()].back();
			const Pt2D& firstStart = validChains[orderedIndices.front()].front();
			float distToFirst = (currEnd - firstStart).length();

			std::optional<size_t> bestIdx;
			float bestDist = std::numeric_limits<float>::max();
			for (size_t i = 0; i < validChains.size(); i++)
			{
				if (used[i])
					continue;
				float d = (currEnd - validChains[i].front()).length();
				if (d < bestDist)
				{
					bestDist = d;
					bestIdx = i;
				}
			}

			if (orderedIndices.size() >= 3 && distToFirst <= bestDist && distToFirst < 1.5f)
				break;
			if (!bestIdx || bestDist > 10.0f)
				break;

			used[*bestIdx] = true;
			orderedIndices.push_back(*bestIdx);
		}

		std::vector<Pt2D> contour;
		for (size_t idx : orderedIndices)
			AppendChainTrimmingOverlap(contour, validChains[idx]);

		// Trim overlap between the end and the start of the closed contour if any
		if (contour.size() > 24)
		{
			constexpr size_t cLookback = 12;
			bool trimmed = false;
			for (size_t i = contour.size() - 1; i > contour.size() - cLookback && !trimmed; i--)
			{
				for (size_t j = 0; j < cLookback; j++)
				{
					if (auto hit = SegmentIntersection2D(contour[i - 1], contour[i], contour[j], contour[j + 1]))
					{
						contour.erase(contour.begin(), contour.begin() + j + 1);
						contour.resize(i - (j + 1));
						contour.push_back(*hit);
						trimmed = true;
						break;
					}
				}
			}
		}

		if (!contour.empty())
		{
			auto minIt = std::min_element(contour.begin(), contour.end(),
				[](const Pt2D& a, const Pt2D& b) { return a.z < b.z; });
			std::rotate(contour.begin(), minIt, contour.end());
		}

		return contour;
	}

	struct HorizontalHit2D
	{
		float x = 0.0f;
		size_t edgeIdx = 0;
		float t = 0.0f;
		Pt2D pt;
	};

	std::vector<HorizontalHit2D> IntersectHorizontalWithPolygon2D(float zp, const std::vector<Pt2D>& poly)
	{
		std::vector<HorizontalHit2D> hits;
		const size_t n = poly.size();
		if (n < 3)
			return hits;

		for (size_t i = 0; i < n; i++)
		{
			const Pt2D& a = poly[i];
			const Pt2D& b = poly[(i + 1) % n];
			if ((a.z <= zp && zp < b.z) || (b.z <= zp && zp < a.z))
			{
				float denom = b.z - a.z;
				if (std::abs(denom) > 1e-9f)
				{
					float t = (zp - a.z) / denom;
					float x = a.x + t * (b.x - a.x);
					hits.push_back({ x, i, t, { x, zp } });
				}
			}
		}

		std::sort(hits.begin(), hits.end(), [](const HorizontalHit2D& a, const HorizontalHit2D& b) {
			return a.x < b.x;
		});
		return hits;
	}

	float ComputePolylineLength2D(const std::vector<Pt2D>& pts)
	{
		float len = 0.0f;
		for (size_t i = 1; i < pts.size(); i++)
			len += (pts[i] - pts[i - 1]).length();
		return len;
	}

	std::vector<Pt2D> ExtractShorterArc2D(
		const std::vector<Pt2D>& poly,
		const HorizontalHit2D& hitA,
		const HorizontalHit2D& hitB)
	{
		const size_t n = poly.size();
		if (n == 0)
			return {};

		std::vector<Pt2D> fwd;
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

		std::vector<Pt2D> bwd;
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
	const float outsideLeftX = -halfX - R - cOutsideBlockMarginMm;
	const float outsideRightX = halfX + R + cOutsideBlockMarginMm;
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

std::vector<float3> PathGenerator::GenerateFlatPath(
	const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
	const FlatMillingParameters& params)
{
	Surface* baseSurf = params.baseSurface;
	if (baseSurf == nullptr)
		return {};

	auto baseEval = GeometryMath::EvaluateParametric(baseSurf, 0.5f, 0.5f);
	const float baseY = std::max(params.baseHeight, baseEval.p.y * cWorldToMm);
	const float maxU2 = static_cast<float>(baseSurf->GetSegmentsU());
	const float maxV2 = static_cast<float>(baseSurf->GetSegmentsV());

	const float safeY = std::max(params.safeHeight, params.blockSizeY + 10.0f);
	const float halfX = params.blockSizeX * 0.5f;
	const float halfZ = params.blockSizeZ * 0.5f;
	const float R = params.cutterDiameter * 0.5f;
	const float outsideLeftX = -halfX - R - cOutsideBlockMarginMm;
	const float outsideRightX = halfX + R + cOutsideBlockMarginMm;
	const float zigMargin = std::max(0.1f, params.zigzagMargin);

	// 1. Find intersections of all non-base model surfaces with the base plane surface
	std::vector<SceneObject*> modelSurfaces;
	for (const auto& obj : sceneObjects)
	{
		if (!obj || obj.get() == baseSurf)
			continue;
		if (obj->IsA(ObjectType::Surface) || obj->type == ObjectType::Torus)
			modelSurfaces.push_back(obj.get());
	}

	std::vector<Poly2D> rawPolys(modelSurfaces.size());
	const float intersectStep = std::clamp(params.intersectionStep, 0.001f, 0.1f);

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < static_cast<int>(modelSurfaces.size()); i++)
	{
		SceneObject* obj = modelSurfaces[i];
		float maxU1 = 1.0f;
		float maxV1 = 1.0f;
		if (auto s1 = obj->As<Surface>())
		{
			maxU1 = static_cast<float>(s1->GetSegmentsU());
			maxV1 = static_cast<float>(s1->GetSegmentsV());
		}
		Vec4f maxDomains(maxU1, maxV1, maxU2, maxV2);
		if (auto interOpt = GeometryMath::CalculateIntersection(obj, baseSurf, false, maxDomains, intersectStep, std::nullopt))
		{
			rawPolys[i] = BuildPolygon2DFromWorldPoints(interOpt->points);
		}
	}

	std::vector<Poly2D> intersectionPolys;
	intersectionPolys.reserve(rawPolys.size());
	for (auto& poly : rawPolys)
	{
		if (poly.pts.size() >= 3)
			intersectionPolys.push_back(std::move(poly));
	}

	// 2. Build outer reachable contours: CL contour (at cutter radius R) and zigzag boundary contour (at R + zigzagMargin)
	std::vector<Pt2D> clContour = BuildOuterReachableContour(intersectionPolys, R, 0.015f);
	std::vector<Pt2D> zigContour = BuildOuterReachableContour(intersectionPolys, R + zigMargin, 0.015f);
	if (zigContour.size() < 3)
		zigContour = clContour;

	const float stepover = std::clamp(params.stepover, 0.5f, params.cutterDiameter * 0.95f);

	std::vector<float3> waypoints;
	waypoints.reserve(1024);

	auto appendWaypoint = [&](float x, float y, float z)
	{
		float3 pt(x, y, z);
		if (waypoints.empty() || (pt.ToVec3f() - waypoints.back().ToVec3f()).length_sqr() > 1e-6f)
			waypoints.push_back(pt);
	};

	auto appendBase = [&](float x, float z)
	{
		appendWaypoint(x, baseY, z);
	};

	if (zigContour.size() < 3)
	{
		// Safety guard: never clear the whole block if model objects exist but failed to produce a contour
		if (!modelSurfaces.empty())
			return {};

		// Fallback: full-block zigzag only when the scene has no model objects on the base plane
		const int numPasses = std::max(1, static_cast<int>(std::ceil(params.blockSizeZ / stepover)));
		appendWaypoint(outsideLeftX, safeY, -halfZ);

		bool leftToRight = true;
		for (int p = 0; p <= numPasses; p++)
		{
			float zp = -halfZ + (params.blockSizeZ * p) / numPasses;
			appendBase(leftToRight ? outsideLeftX : outsideRightX, zp);
			appendBase(leftToRight ? outsideRightX : outsideLeftX, zp);
			leftToRight = !leftToRight;
		}
		appendWaypoint(waypoints.back().x, safeY, waypoints.back().z);
		return waypoints;
	}

	// 3. Plan uniform Z scanlines across the block respecting the exact requested stepover
	const int numPasses = std::max(1, static_cast<int>(std::ceil(params.blockSizeZ / stepover)));
	std::vector<float> zPasses(numPasses + 1);
	for (int p = 0; p <= numPasses; p++)
		zPasses[p] = -halfZ + (params.blockSizeZ * p) / numPasses;

	const float zMinObj = zigContour.front().z;

	std::vector<float> topZ;
	std::vector<float> midZ;
	std::vector<float> botZ;
	for (float zp : zPasses)
	{
		auto hits = IntersectHorizontalWithPolygon2D(zp, zigContour);
		if (hits.empty())
		{
			if (zp < zMinObj)
				topZ.push_back(zp);
			else
				botZ.push_back(zp);
		}
		else
		{
			midZ.push_back(zp);
		}
	}

	// 4. Generate continuous non-backtracking zigzag starting each zone from where the previous zone ended
	const float firstZ = zPasses.front();
	appendWaypoint(outsideLeftX, safeY, firstZ);

	// 4a. Top full-width passes (starts on the left: currRight == false)
	bool currRight = false;
	for (float zp : topZ)
	{
		appendBase(currRight ? outsideRightX : outsideLeftX, zp);
		appendBase(currRight ? outsideLeftX : outsideRightX, zp);
		currRight = !currRight;
	}

	// If midZ has an odd count and midZ[0] cleanly spans the top tip, bridge across midZ[0] to pair the remaining mid lines
	std::vector<float> remMidZ = midZ;
	if (remMidZ.size() % 2 == 1)
	{
		auto hits0 = IntersectHorizontalWithPolygon2D(remMidZ.front(), zigContour);
		if (hits0.size() >= 2)
		{
			float z0 = remMidZ.front();
			remMidZ.erase(remMidZ.begin());

			const HorizontalHit2D& hStart = currRight ? hits0.back() : hits0.front();
			const HorizontalHit2D& hEnd = currRight ? hits0.front() : hits0.back();

			appendBase(currRight ? outsideRightX : outsideLeftX, z0);
			appendBase(hStart.pt.x, hStart.pt.z);

			std::vector<Pt2D> arc = ExtractShorterArc2D(zigContour, hStart, hEnd);
			std::vector<Pt2D> simpArc = SimplifyPolyline2D(arc, params.simplifyTolerance);
			for (size_t m = 1; m < simpArc.size(); m++)
				appendBase(simpArc[m].x, simpArc[m].z);

			appendBase(currRight ? outsideLeftX : outsideRightX, z0);
			currRight = !currRight;
		}
	}

	auto traceSideZigzag = [&](const std::vector<float>& zList, bool isRightSide)
	{
		float outX = isRightSide ? outsideRightX : outsideLeftX;
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

			appendBase(outX, zA);
			appendBase(hitA.pt.x, hitA.pt.z);

			std::vector<Pt2D> arc = ExtractShorterArc2D(zigContour, hitA, hitB);
			std::vector<Pt2D> simpArc = SimplifyPolyline2D(arc, params.simplifyTolerance);
			for (size_t m = 1; m < simpArc.size(); m++)
				appendBase(simpArc[m].x, simpArc[m].z);

			appendBase(outX, zB);
		}
	};

	// 4b. First descending side along remMidZ (pairs keep cutter on currRight)
	traceSideZigzag(remMidZ, currRight);

	// If remMidZ remained odd (fallback when bridge wasn't taken), handle the final unpaired line cleanly
	if (remMidZ.size() % 2 == 1)
	{
		float zLast = remMidZ.back();
		auto hitsLast = IntersectHorizontalWithPolygon2D(zLast, zigContour);
		if (!hitsLast.empty())
		{
			const HorizontalHit2D& hLast = currRight ? hitsLast.back() : hitsLast.front();
			float outX = currRight ? outsideRightX : outsideLeftX;
			appendBase(outX, zLast);
			appendBase(hLast.pt.x, zLast);
			appendBase(outX, zLast);
		}
	}

	// 4c. Bottom passes crossing from currRight to !currRight
	std::vector<std::pair<HorizontalHit2D, HorizontalHit2D>> midOpenBays;
	if (!midZ.empty())
	{
		auto lastMidHits = IntersectHorizontalWithPolygon2D(midZ.back(), zigContour);
		if (lastMidHits.size() >= 4)
		{
			for (size_t m = 1; m + 1 < lastMidHits.size(); m += 2)
				midOpenBays.emplace_back(lastMidHits[m], lastMidHits[m + 1]);
		}
	}

	auto appendBaysAlongLine = [&](float zp, bool ltr)
	{
		if (midOpenBays.empty())
			return;

		if (ltr)
		{
			for (const auto& [hLeft, hRight] : midOpenBays)
			{
				appendBase(hRight.pt.x, zp);
				appendBase(hRight.pt.x, hRight.pt.z);
				appendBase(hLeft.pt.x, hLeft.pt.z);
				appendBase(hLeft.pt.x, zp);
			}
		}
		else
		{
			for (auto it = midOpenBays.rbegin(); it != midOpenBays.rend(); ++it)
			{
				const auto& [hLeft, hRight] = *it;
				appendBase(hLeft.pt.x, zp);
				appendBase(hLeft.pt.x, hLeft.pt.z);
				appendBase(hRight.pt.x, hRight.pt.z);
				appendBase(hRight.pt.x, zp);
			}
		}
	};

	if (!botZ.empty())
	{
		bool botLtr = !currRight;
		for (size_t idxB = 0; idxB < botZ.size(); idxB++)
		{
			float zp = botZ[idxB];
			float startX = botLtr ? outsideLeftX : outsideRightX;
			float endX = botLtr ? outsideRightX : outsideLeftX;

			appendBase(startX, zp);
			if (idxB == 0)
				appendBaysAlongLine(zp, botLtr);
			appendBase(endX, zp);
			botLtr = !botLtr;
		}

		// If botZ had an even count, step outside the bottom edge of the block to switch sides without backtracking
		if (botZ.size() % 2 == 0)
		{
			float outsideBotZ = std::max(halfZ, botZ.back()) + R + cOutsideBlockMarginMm;
			appendBase(waypoints.back().x, outsideBotZ);
			appendBase(currRight ? outsideLeftX : outsideRightX, outsideBotZ);
		}
	}
	else
	{
		float outsideBotZ = halfZ + R + cOutsideBlockMarginMm;
		appendBase(waypoints.back().x, outsideBotZ);
		appendBaysAlongLine(outsideBotZ, !currRight);
		appendBase(currRight ? outsideLeftX : outsideRightX, outsideBotZ);
	}

	// 4d. Opposite ascending side going bottom-to-top
	std::vector<float> revMidZ(remMidZ.rbegin(), remMidZ.rend());
	traceSideZigzag(revMidZ, !currRight);
	if (remMidZ.size() % 2 == 1)
	{
		float zTop = revMidZ.back();
		auto hitsTop = IntersectHorizontalWithPolygon2D(zTop, zigContour);
		if (!hitsTop.empty())
		{
			const HorizontalHit2D& hTop = (!currRight) ? hitsTop.back() : hitsTop.front();
			float outX = (!currRight) ? outsideRightX : outsideLeftX;
			appendBase(outX, zTop);
			appendBase(hTop.pt.x, zTop);
			appendBase(outX, zTop);
		}
	}

	// 5. Enter clContour cleanly between the ending height and contour apex, trace full contour, then exit outside block
	if (clContour.size() >= 3)
	{
		const float3 endPt = waypoints.back();
		float zEntry = 0.5f * (endPt.z + clContour.front().z);
		auto hitsCl = IntersectHorizontalWithPolygon2D(zEntry, clContour);
		if (!hitsCl.empty())
		{
			bool enterFromRight = (endPt.x > 0.0f);
			const HorizontalHit2D& entryHit = enterFromRight ? hitsCl.back() : hitsCl.front();

			appendBase(endPt.x, zEntry);
			appendBase(entryHit.pt.x, entryHit.pt.z);

			std::vector<Pt2D> clRotated;
			clRotated.reserve(clContour.size() + 2);
			clRotated.push_back(entryHit.pt);
			const size_t startIdx = (entryHit.edgeIdx + 1) % clContour.size();
			for (size_t k = 0; k < clContour.size(); k++)
				clRotated.push_back(clContour[(startIdx + k) % clContour.size()]);
			clRotated.push_back(entryHit.pt);

			std::vector<Pt2D> simplifiedContour = SimplifyPolyline2D(clRotated, params.simplifyTolerance);
			for (size_t k = 1; k < simplifiedContour.size(); k++)
				appendBase(simplifiedContour[k].x, simplifiedContour[k].z);

			float exitX = enterFromRight ? outsideRightX : outsideLeftX;
			float exitOffsetZ = (zEntry >= clContour.front().z) ? -1.5f : 1.5f;
			appendBase(exitX, entryHit.pt.z + exitOffsetZ);
		}
		else
		{
			std::vector<Pt2D> closedContour = clContour;
			closedContour.push_back(clContour.front());
			std::vector<Pt2D> simplifiedContour = SimplifyPolyline2D(closedContour, params.simplifyTolerance);

			if (simplifiedContour.size() >= 3)
			{
				appendBase(endPt.x, simplifiedContour.front().z);
				for (const auto& pt : simplifiedContour)
					appendBase(pt.x, pt.z);

				float dxExit = simplifiedContour.back().x - simplifiedContour[simplifiedContour.size() - 2].x;
				float exitX = (dxExit < 0.0f) ? outsideLeftX : outsideRightX;
				appendBase(exitX, simplifiedContour.back().z);
			}
		}
	}

	// 6. Final retract in air outside the block to safe height
	if (!waypoints.empty())
	{
		const float3& lastPt = waypoints.back();
		appendWaypoint(lastPt.x, safeY, lastPt.z);
	}

	return waypoints;
}

namespace
{
	struct FinishingSurfaceRuntime
	{
		SceneObject* obj = nullptr;
		unsigned int id = 0;
		bool isUDirection = true;
		float maxU = 1.0f;
		float maxV = 1.0f;
		bool wrapU = false;
		bool wrapV = false;
		float normalSign = 1.0f;
	};

	struct SurfaceSampleMm
	{
		Vec3f posMm{};
		Vec3f duMm{};
		Vec3f dvMm{};
		Vec3f normal{};
		Vec3f ballCenterMm{};
		Vec3f tipMm{};
	};

	struct UVSeg2D
	{
		Vec2f a{};
		Vec2f b{};
	};

	struct IsoPassSegment
	{
		float w = 0.0f;
		float tStart = 0.0f;
		float tEnd = 0.0f;
		std::vector<float3> pts; // cutter tip waypoints in scene mm
	};

	void GetObjectDomainAndWrapping(
		SceneObject* obj,
		float& outMaxU,
		float& outMaxV,
		bool& outWrapU,
		bool& outWrapV)
	{
		outMaxU = 1.0f;
		outMaxV = 1.0f;
		outWrapU = false;
		outWrapV = false;
		if (!obj)
			return;

		if (auto surf = obj->As<Surface>())
		{
			outMaxU = static_cast<float>(std::max(1u, surf->GetSegmentsU()));
			outMaxV = static_cast<float>(std::max(1u, surf->GetSegmentsV()));
			Vec3f u0 = GeometryMath::EvaluateParametric(obj, 0.0f, 0.5f * outMaxV).p;
			Vec3f u1 = GeometryMath::EvaluateParametric(obj, outMaxU, 0.5f * outMaxV).p;
			outWrapU = ((u0 - u1).length_sqr() < 1e-6f);

			Vec3f v0 = GeometryMath::EvaluateParametric(obj, 0.5f * outMaxU, 0.0f).p;
			Vec3f v1 = GeometryMath::EvaluateParametric(obj, 0.5f * outMaxU, outMaxV).p;
			outWrapV = ((v0 - v1).length_sqr() < 1e-6f);
		}
		else if (obj->type == ObjectType::Torus)
		{
			outWrapU = true;
			outWrapV = true;
		}
	}

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

	float DetermineOutwardNormalSign(
		SceneObject* obj,
		float maxU,
		float maxV,
		float baseY)
	{
		constexpr int cGrid = 32;
		float maxSurfY = -std::numeric_limits<float>::max();

		for (int iv = 0; iv <= cGrid; iv++)
		{
			float v = (maxV * iv) / cGrid;
			for (int iu = 0; iu <= cGrid; iu++)
			{
				float u = (maxU * iu) / cGrid;
				auto eval = GeometryMath::EvaluateParametric(obj, u, v);
				float yMm = eval.p.y * cWorldToMm;
				maxSurfY = std::max(maxSurfY, yMm);
			}
		}

		// Accumulate normal Y near the highest visible crest of the surface (where outward normal must point +Y)
		const float crestThreshold = std::max(baseY + 0.1f, maxSurfY - 6.0f);
		double weightedNySum = 0.0;

		for (int iv = 0; iv <= cGrid; iv++)
		{
			float v = (maxV * iv) / cGrid;
			for (int iu = 0; iu <= cGrid; iu++)
			{
				float u = (maxU * iu) / cGrid;
				auto eval = GeometryMath::EvaluateParametric(obj, u, v);
				float yMm = eval.p.y * cWorldToMm;
				if (yMm < baseY)
					continue;

				Vec3f crossUV = Vec3f::cross(eval.du, eval.dv);
				float len = crossUV.length();
				if (len < 1e-7f)
					continue;

				float ny = crossUV.y / len;
				float heightAboveBase = std::max(0.0f, yMm - baseY);
				double weight = static_cast<double>(heightAboveBase) * heightAboveBase;
				if (yMm >= crestThreshold)
					weight *= 25.0;

				weightedNySum += static_cast<double>(ny) * weight;
			}
		}

		return (weightedNySum < 0.0) ? -1.0f : 1.0f;
	}

	SurfaceSampleMm EvaluateSurfaceSampleMm(
		const FinishingSurfaceRuntime& surf,
		float u,
		float v,
		float R)
	{
		float uu = WrapOrClampParam(u, surf.maxU, surf.wrapU);
		float vv = WrapOrClampParam(v, surf.maxV, surf.wrapV);
		auto eval = GeometryMath::EvaluateParametric(surf.obj, uu, vv);

		SurfaceSampleMm sample;
		sample.posMm = eval.p * cWorldToMm;
		sample.duMm = eval.du * cWorldToMm;
		sample.dvMm = eval.dv * cWorldToMm;

		Vec3f rawN = Vec3f::cross(sample.duMm, sample.dvMm) * surf.normalSign;
		float nLen = rawN.length();
		if (nLen > 1e-6f)
			sample.normal = rawN * (1.0f / nLen);
		else
			sample.normal = Vec3f(0.0f, 1.0f, 0.0f);

		sample.ballCenterMm = sample.posMm + sample.normal * R;
		sample.tipMm = sample.ballCenterMm - Vec3f(0.0f, R, 0.0f);
		return sample;
	}

	std::vector<float> BuildSingleSurfaceHeightMap(
		SceneObject* obj,
		float blockSizeX,
		float blockSizeZ,
		int divX,
		int divZ)
	{
		const int cols = divX + 1;
		const int rows = divZ + 1;
		const float halfX = blockSizeX * 0.5f;
		const float halfZ = blockSizeZ * 0.5f;
		const float dx = blockSizeX / divX;
		const float dz = blockSizeZ / divZ;

		std::vector<float> heightGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());
		if (!obj)
			return heightGrid;

		std::vector<SegmentTask> tasks;
		if (auto surf = obj->As<Surface>())
		{
			unsigned int segU = surf->GetSegmentsU();
			unsigned int segV = surf->GetSegmentsV();
			for (unsigned int sv = 0; sv < segV; sv++)
			{
				for (unsigned int su = 0; su < segU; su++)
				{
					tasks.push_back({
						obj,
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
					tasks.push_back({ obj, u0, u1, v0, v1 });
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
				divX, divZ,
				heightGrid);
		}

		return heightGrid;
	}

	// Builds an exact-radius (cellHalfDiag = 0, surfaceMargin = 0) Inverse Tool Offset height map
	// for a single surface so that adjacent surface collision checks never suffer from artificial thickening.
	std::vector<float> BuildExactSingleSurfaceItoMap(
		const std::vector<float>& surfaceHeightGrid,
		float blockSizeX,
		float blockSizeZ,
		int divX,
		int divZ,
		float R,
		float baseY,
		const std::vector<KernelOffset>& exactKernel)
	{
		const int cols = divX + 1;
		const int rows = divZ + 1;
		std::vector<float> itoGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());

		#pragma omp parallel for schedule(dynamic, 8)
		for (int jz = 0; jz <= divZ; jz++)
		{
			size_t srcRowOffset = static_cast<size_t>(jz) * cols;
			for (int jx = 0; jx <= divX; jx++)
			{
				float rawH = surfaceHeightGrid[srcRowOffset + jx];
				if (rawH < baseY - 0.5f)
					continue;

				for (const auto& k : exactKernel)
				{
					int ix = jx + k.ox;
					int iz = jz + k.oz;
					if (ix >= 0 && ix <= divX && iz >= 0 && iz <= divZ)
					{
						float requiredTipY = rawH + k.sphereHeightOffset;
						size_t dstIdx = static_cast<size_t>(iz) * cols + ix;
						if (requiredTipY > itoGrid[dstIdx])
						{
							AtomicMaxFloat(&itoGrid[dstIdx], requiredTipY);
						}
					}
				}
			}
		}

		return itoGrid;
	}

	float SampleGridBilinearMax(
		const std::vector<float>& grid,
		float xMm,
		float zMm,
		float halfX,
		float halfZ,
		float invDx,
		float invDz,
		int divX,
		int divZ)
	{
		const int cols = divX + 1;
		float fx = (xMm + halfX) * invDx;
		float fz = (zMm + halfZ) * invDz;

		int ix0 = std::clamp(static_cast<int>(std::floor(fx)), 0, divX);
		int ix1 = std::clamp(ix0 + 1, 0, divX);
		int iz0 = std::clamp(static_cast<int>(std::floor(fz)), 0, divZ);
		int iz1 = std::clamp(iz0 + 1, 0, divZ);

		float tx = std::clamp(fx - static_cast<float>(ix0), 0.0f, 1.0f);
		float tz = std::clamp(fz - static_cast<float>(iz0), 0.0f, 1.0f);

		float h00 = grid[static_cast<size_t>(iz0) * cols + ix0];
		float h10 = grid[static_cast<size_t>(iz0) * cols + ix1];
		float h01 = grid[static_cast<size_t>(iz1) * cols + ix0];
		float h11 = grid[static_cast<size_t>(iz1) * cols + ix1];

		if (h00 < -1e5f || h10 < -1e5f || h01 < -1e5f || h11 < -1e5f)
		{
			return std::max({ h00, h10, h01, h11 });
		}

		float h0 = std::lerp(h00, h10, tx);
		float h1 = std::lerp(h01, h11, tx);
		return std::lerp(h0, h1, tz);
	}

	void AddUVChainToSegments(
		const std::vector<Vec2f>& rawUVs,
		bool isLooped,
		float maxU,
		float maxV,
		bool wrapU,
		bool wrapV,
		std::vector<UVSeg2D>& outSegments)
	{
		if (rawUVs.size() < 2)
			return;

		const size_t n = rawUVs.size();
		const size_t segCount = isLooped ? n : (n - 1);

		for (size_t i = 0; i < segCount; i++)
		{
			Vec2f p0 = rawUVs[i];
			Vec2f p1 = rawUVs[(i + 1) % n];

			p0.x = WrapOrClampParam(p0.x, maxU, wrapU);
			p0.y = WrapOrClampParam(p0.y, maxV, wrapV);
			p1.x = WrapOrClampParam(p1.x, maxU, wrapU);
			p1.y = WrapOrClampParam(p1.y, maxV, wrapV);

			float du = p1.x - p0.x;
			float dv = p1.y - p0.y;

			if (wrapU && maxU > 0.0f)
			{
				if (du > 0.5f * maxU) du -= maxU;
				else if (du < -0.5f * maxU) du += maxU;
			}
			if (wrapV && maxV > 0.0f)
			{
				if (dv > 0.5f * maxV) dv -= maxV;
				else if (dv < -0.5f * maxV) dv += maxV;
			}

			// Ignore spurious long jumps
			if (std::abs(du) > 0.35f * maxU || std::abs(dv) > 0.35f * maxV)
				continue;

			Vec2f q1(p0.x + du, p0.y + dv);
			outSegments.push_back({ p0, q1 });

			// Replicate segment across periodic boundary if q1 stepped outside [0, maxU] or [0, maxV]
			float shiftU = 0.0f;
			if (wrapU)
			{
				if (q1.x < 0.0f) shiftU = maxU;
				else if (q1.x > maxU) shiftU = -maxU;
			}
			float shiftV = 0.0f;
			if (wrapV)
			{
				if (q1.y < 0.0f) shiftV = maxV;
				else if (q1.y > maxV) shiftV = -maxV;
			}

			if (shiftU != 0.0f || shiftV != 0.0f)
			{
				Vec2f shift(shiftU, shiftV);
				outSegments.push_back({ p0 + shift, q1 + shift });
			}
		}
	}

	std::vector<float> CollectIsoLineUVHits(
		float w,
		bool isUDirection,
		float maxT,
		bool wrapT,
		const std::vector<UVSeg2D>& uvSegments)
	{
		std::vector<float> hits;
		hits.push_back(0.0f);
		hits.push_back(maxT);

		// Pass direction t is assumed to run parallel to any C0 patch seams (C0 edges occur only at constant w = k).
		for (const auto& seg : uvSegments)
		{
			// Pass parameter t = u (if isUDirection) else v; cross parameter w = v (if isUDirection) else u
			float t0 = isUDirection ? seg.a.x : seg.a.y;
			float w0 = isUDirection ? seg.a.y : seg.a.x;
			float t1 = isUDirection ? seg.b.x : seg.b.y;
			float w1 = isUDirection ? seg.b.y : seg.b.x;

			float minW = std::min(w0, w1);
			float maxW = std::max(w0, w1);
			if (w < minW - 1e-6f || w > maxW + 1e-6f)
				continue;

			float dw = w1 - w0;
			if (std::abs(dw) > 1e-7f)
			{
				float alpha = (w - w0) / dw;
				if (alpha >= -1e-4f && alpha <= 1.0001f)
				{
					alpha = std::clamp(alpha, 0.0f, 1.0f);
					float hitT = WrapOrClampParam(t0 + alpha * (t1 - t0), maxT, wrapT);
					hits.push_back(hitT);
				}
			}
		}

		std::sort(hits.begin(), hits.end());
		std::vector<float> uniqueHits;
		uniqueHits.reserve(hits.size());
		for (float h : hits)
		{
			h = std::clamp(h, 0.0f, maxT);
			if (uniqueHits.empty() || std::abs(h - uniqueHits.back()) > 1e-4f)
				uniqueHits.push_back(h);
		}
		if (uniqueHits.empty() || uniqueHits.front() > 1e-4f)
			uniqueHits.insert(uniqueHits.begin(), 0.0f);
		if (uniqueHits.back() < maxT - 1e-4f)
			uniqueHits.push_back(maxT);
		else
			uniqueHits.back() = maxT;

		return uniqueHits;
	}

	// Gouge-safe 3D chordal polyline simplification for curved finishing passes
	std::vector<float3> SimplifyPolyline3D(const std::vector<float3>& rawPts, float tolerance)
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
}

bool PathGenerator::LoadFinishingConfigFromJson(
	const std::string& jsonFilepath,
	std::vector<FinishingSurfaceConfig>& outSurfaces,
	std::vector<FinishingIntersectionPairConfig>& outIntersections)
{
	outSurfaces.clear();
	outIntersections.clear();

	std::ifstream file(jsonFilepath);
	if (!file.is_open())
		return false;

	try
	{
		nlohmann::json j;
		file >> j;

		if (j.contains("surfaces") && j["surfaces"].is_array())
		{
			for (const auto& s : j["surfaces"])
			{
				if (!s.contains("id"))
					continue;

				FinishingSurfaceConfig cfg;
				cfg.surfaceId = s["id"].get<unsigned int>();
				cfg.isUDirection = true;
				if (s.contains("isoDirection") && s["isoDirection"].is_string())
				{
					std::string dir = s["isoDirection"].get<std::string>();
					if (!dir.empty() && (dir[0] == 'v' || dir[0] == 'V'))
						cfg.isUDirection = false;
				}
				outSurfaces.push_back(cfg);
			}
		}

		if (j.contains("intersections") && j["intersections"].is_array())
		{
			for (const auto& inter : j["intersections"])
			{
				if (!inter.contains("idA") || !inter.contains("idB"))
					continue;

				FinishingIntersectionPairConfig pairCfg;
				pairCfg.idA = inter["idA"].get<unsigned int>();
				pairCfg.idB = inter["idB"].get<unsigned int>();

				if (inter.contains("cursorSeeds") && inter["cursorSeeds"].is_array())
				{
					for (const auto& seed : inter["cursorSeeds"])
					{
						if (seed.is_array() && seed.size() >= 3)
						{
							pairCfg.cursorSeeds.emplace_back(
								seed[0].get<float>(),
								seed[1].get<float>(),
								seed[2].get<float>());
						}
					}
				}
				outIntersections.push_back(std::move(pairCfg));
			}
		}
	}
	catch (const std::exception&)
	{
		return false;
	}

	return !outSurfaces.empty();
}

std::vector<float3> PathGenerator::GenerateFinishingPath(
	const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
	const FinishingParameters& params)
{
	FinishingParameters effectiveParams = params;

	// 1. Load configuration from JSON if not already populated in params
	if (effectiveParams.surfaces.empty())
	{
		std::vector<std::string> candidatePaths;
		if (!effectiveParams.configJsonPath.empty())
			candidatePaths.push_back(effectiveParams.configJsonPath);
		candidatePaths.push_back("intersections.json");
		candidatePaths.push_back("../intersections.json");
		candidatePaths.push_back("intersections_v.json");
		candidatePaths.push_back("../intersections_v.json");

		for (const auto& path : candidatePaths)
		{
			if (std::filesystem::exists(path))
			{
				if (LoadFinishingConfigFromJson(path, effectiveParams.surfaces, effectiveParams.intersections))
					break;
			}
		}
	}

	// Determine base surface and base height (minimum 15.0 mm)
	Surface* baseSurf = effectiveParams.baseSurface;
	if (baseSurf == nullptr)
	{
		for (const auto& obj : sceneObjects)
		{
			if (obj && obj->IsA(ObjectType::Surface))
			{
				std::string lowerName = obj->name;
				std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
					[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				if (lowerName == "plane" || lowerName.find("plane") != std::string::npos)
				{
					baseSurf = obj->As<Surface>();
					break;
				}
			}
		}
	}

	float baseY = effectiveParams.baseHeight;
	if (baseSurf != nullptr)
	{
		auto baseEval = GeometryMath::EvaluateParametric(baseSurf, 0.5f, 0.5f);
		baseY = std::max(baseY, baseEval.p.y * cWorldToMm);
	}
	const float minSafeTipY = baseY + 0.015f; // Strict margin above 15.0 mm base height

	const float R = effectiveParams.cutterDiameter * 0.5f; // 4.0 mm for K08
	const float safeY = std::max(effectiveParams.safeHeight, effectiveParams.blockSizeY + 10.0f);
	const float halfX = effectiveParams.blockSizeX * 0.5f;
	const float halfZ = effectiveParams.blockSizeZ * 0.5f;
	const int divX = std::max(64, effectiveParams.gridDivX);
	const int divZ = std::max(64, effectiveParams.gridDivZ);
	const int cols = divX + 1;
	const int rows = divZ + 1;
	const float dx = effectiveParams.blockSizeX / divX;
	const float dz = effectiveParams.blockSizeZ / divZ;
	const float invDx = 1.0f / dx;
	const float invDz = 1.0f / dz;

	// 2. Collect model surfaces in the order specified by effectiveParams.surfaces
	std::vector<FinishingSurfaceRuntime> modelSurfaces;
	modelSurfaces.reserve(effectiveParams.surfaces.size());
	for (const auto& cfg : effectiveParams.surfaces)
	{
		for (const auto& obj : sceneObjects)
		{
			if (!obj || obj.get() == baseSurf || obj->m_id != cfg.surfaceId)
				continue;
			if (!obj->IsA(ObjectType::Surface) && obj->type != ObjectType::Torus)
				continue;

			FinishingSurfaceRuntime rt;
			rt.obj = obj.get();
			rt.id = obj->m_id;
			rt.isUDirection = cfg.isUDirection;
			GetObjectDomainAndWrapping(rt.obj, rt.maxU, rt.maxV, rt.wrapU, rt.wrapV);
			rt.normalSign = DetermineOutwardNormalSign(rt.obj, rt.maxU, rt.maxV, baseY);
			modelSurfaces.push_back(rt);
			break;
		}
	}

	if (modelSurfaces.empty())
		return {};

	// 3. Build per-surface height maps and ITO maps
	const int radCellsX = static_cast<int>(std::ceil(R / dx));
	const int radCellsZ = static_cast<int>(std::ceil(R / dz));
	const float R2 = R * R;

	std::vector<KernelOffset> exactKernel;
	exactKernel.reserve((static_cast<size_t>(2) * radCellsX + 1) * (static_cast<size_t>(2) * radCellsZ + 1));
	for (int oz = -radCellsZ; oz <= radCellsZ; oz++)
	{
		float distZ = oz * dz;
		for (int ox = -radCellsX; ox <= radCellsX; ox++)
		{
			float distX = ox * dx;
			float distSq = distX * distX + distZ * distZ;
			if (distSq <= R2)
			{
				float sphereOffset = -R + std::sqrt(std::max(0.0f, R2 - distSq));
				exactKernel.push_back({ ox, oz, sphereOffset });
			}
		}
	}

	const size_t numSurf = modelSurfaces.size();
	std::vector<std::vector<float>> surfGrids(numSurf);
	std::vector<std::vector<float>> itoGrids(numSurf);
	for (size_t m = 0; m < numSurf; m++)
	{
		surfGrids[m] = BuildSingleSurfaceHeightMap(
			modelSurfaces[m].obj,
			effectiveParams.blockSizeX,
			effectiveParams.blockSizeZ,
			divX, divZ);

		itoGrids[m] = BuildExactSingleSurfaceItoMap(
			surfGrids[m],
			effectiveParams.blockSizeX,
			effectiveParams.blockSizeZ,
			divX, divZ,
			R, baseY, exactKernel);
	}

	// Global combined ITO grid across all surfaces + highest roughed-stock clearance ceiling
	std::vector<float> globalItoGrid(static_cast<size_t>(cols) * rows, minSafeTipY);
	float maxModelSurfY = baseY;
	for (size_t m = 0; m < numSurf; m++)
	{
		for (size_t cell = 0; cell < globalItoGrid.size(); cell++)
		{
			globalItoGrid[cell] = std::max(globalItoGrid[cell], itoGrids[m][cell]);
			maxModelSurfY = std::max(maxModelSurfY, surfGrids[m][cell]);
		}
	}
	// Highest point of the model after roughing (model peak + roughing margin 0.5 mm + K16 cusp ~1.0 mm + safety margin 1.5 mm)
	const float globalModelClearY = std::min(safeY, std::max(minSafeTipY + 5.0f, maxModelSurfY + 3.0f));

	// 4. Compute exact surface-surface and surface-base intersections
	const float intersectStep = std::clamp(effectiveParams.intersectionStep, 0.002f, 0.05f);
	std::vector<std::vector<UVSeg2D>> surfaceUVTrimSegments(numSurf);

	struct GrooveCurveRecord
	{
		size_t idxA = 0;
		size_t idxB = 0;
		GeometryMath::IntersectionResult data;
	};
	std::vector<GrooveCurveRecord> grooveCurves;

	auto findSurfaceIndexById = [&](unsigned int id) -> std::optional<size_t>
	{
		for (size_t i = 0; i < numSurf; i++)
		{
			if (modelSurfaces[i].id == id)
				return i;
		}
		return std::nullopt;
	};

	// 4a. Surface-Surface intersections from JSON cursorSeeds
	for (const auto& pairCfg : effectiveParams.intersections)
	{
		auto idxAOpt = findSurfaceIndexById(pairCfg.idA);
		auto idxBOpt = findSurfaceIndexById(pairCfg.idB);
		if (!idxAOpt || !idxBOpt)
			continue;

		const size_t idxA = *idxAOpt;
		const size_t idxB = *idxBOpt;
		const auto& surfA = modelSurfaces[idxA];
		const auto& surfB = modelSurfaces[idxB];
		Vec4f maxDomains(surfA.maxU, surfA.maxV, surfB.maxU, surfB.maxV);

		auto processPairIntersection = [&](std::optional<float3> seedOpt)
		{
			if (auto interOpt = GeometryMath::CalculateIntersection(
					surfA.obj, surfB.obj, (surfA.obj == surfB.obj), maxDomains, intersectStep, seedOpt))
			{
				if (interOpt->points.size() >= 2)
				{
					std::vector<Vec2f> uvA;
					std::vector<Vec2f> uvB;
					uvA.reserve(interOpt->params.size());
					uvB.reserve(interOpt->params.size());
					for (const auto& uv : interOpt->params)
					{
						uvA.emplace_back(uv.x, uv.y);
						uvB.emplace_back(uv.z, uv.w);
					}

					AddUVChainToSegments(uvA, interOpt->isClosedLoop, surfA.maxU, surfA.maxV, surfA.wrapU, surfA.wrapV, surfaceUVTrimSegments[idxA]);
					AddUVChainToSegments(uvB, interOpt->isClosedLoop, surfB.maxU, surfB.maxV, surfB.wrapU, surfB.wrapV, surfaceUVTrimSegments[idxB]);

					GrooveCurveRecord rec;
					rec.idxA = idxA;
					rec.idxB = idxB;
					rec.data = std::move(*interOpt);
					grooveCurves.push_back(std::move(rec));
				}
			}
		};

		if (!pairCfg.cursorSeeds.empty())
		{
			for (const auto& seed : pairCfg.cursorSeeds)
				processPairIntersection(seed);
		}
		else
		{
			processPairIntersection(std::nullopt);
		}
	}

	// 4b. Surface-Base intersections (all model surfaces against baseSurface without cursor seed)
	if (baseSurf != nullptr)
	{
		const float maxU2 = static_cast<float>(std::max(1u, baseSurf->GetSegmentsU()));
		const float maxV2 = static_cast<float>(std::max(1u, baseSurf->GetSegmentsV()));

		for (size_t m = 0; m < numSurf; m++)
		{
			const auto& surfM = modelSurfaces[m];
			Vec4f maxDomains(surfM.maxU, surfM.maxV, maxU2, maxV2);
			if (auto interOpt = GeometryMath::CalculateIntersection(
					surfM.obj, baseSurf, false, maxDomains, intersectStep, std::nullopt))
			{
				if (interOpt->points.size() >= 2)
				{
					std::vector<Vec2f> uvM;
					uvM.reserve(interOpt->params.size());
					for (const auto& uv : interOpt->params)
						uvM.emplace_back(uv.x, uv.y);

					AddUVChainToSegments(uvM, interOpt->isClosedLoop, surfM.maxU, surfM.maxV, surfM.wrapU, surfM.wrapV, surfaceUVTrimSegments[m]);
				}
			}
		}
	}

	// 5. Helper to append waypoints with protection against Error_VerticalDownward
	std::vector<float3> waypoints;
	waypoints.reserve(18000);

	constexpr float cMinSafeHorizontalSlope = 0.048f; // ~2.75 deg from vertical (> 1.5 deg simulator threshold)

	auto appendSafePoint = [&](const float3& targetPt)
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
		const float distSq = (pt.ToVec3f() - prev).length_sqr();
		if (distSq < 1e-6f)
			return;

		// Guard against near-vertical downward segments (< 1.5 deg from vertical) without ever lifting Y via globalItoGrid
		if (dyMove < -1e-4f)
		{
			const float lenXZ = std::sqrt(dxMove * dxMove + dzMove * dzMove);
			const float minRequiredXZ = (-dyMove) * cMinSafeHorizontalSlope;
			if (lenXZ < minRequiredXZ)
			{
				float dirX = (lenXZ > 1e-5f) ? (dxMove / lenXZ) : 1.0f;
				float dirZ = (lenXZ > 1e-5f) ? (dzMove / lenXZ) : 0.0f;
				pt.x = prev.x + dirX * minRequiredXZ;
				pt.z = prev.z + dirZ * minRequiredXZ;
			}
		}

		waypoints.push_back(pt);
	};

	auto isStraightSegmentCollisionFree = [&](const Vec3f& a, const Vec3f& b, const std::vector<float>& grid, float tol) -> bool
	{
		if (std::min(a.y, b.y) < minSafeTipY - 0.01f)
			return false;
		const float dist = (b - a).length();
		if (dist < 1e-5f)
			return true;
		const int steps = std::clamp(static_cast<int>(std::ceil(dist / 0.40f)), 2, 128);
		for (int s = 1; s < steps; s++)
		{
			const float alpha = static_cast<float>(s) / steps;
			const Vec3f pt = Vec3f::lerp(a, b, alpha);
			const float reqH = SampleGridBilinearMax(grid, pt.x, pt.z, halfX, halfZ, invDx, invDz, divX, divZ);
			if (pt.y < reqH - tol)
				return false;
		}
		return true;
	};

	// Transfer using local CL clearance (only slightly above globalItoGrid along the transfer path,
	// capped by globalModelClearY; initial entry still plunges safely from safeY).
	auto transitionSafeCl = [&](const float3& targetPt)
	{
		float3 dest = targetPt;
		dest.y = std::max(dest.y, minSafeTipY);

		float travelY = safeY;
		if (waypoints.empty())
		{
			// Initial entry: start at safeY outside the block, plunge vertically in empty air
			// down to an approach clearance height, traverse horizontally to a side lead-in point,
			// and glide into dest via a shallow lateral ramp (never plunge near-vertically into block).
			struct SideCandidate { float x, z; };
			const float padEdge = R + 2.5f;
			const SideCandidate candidates[4] = {
				{ +halfX + padEdge, dest.z },
				{ -halfX - padEdge, dest.z },
				{ dest.x, +halfZ + padEdge },
				{ dest.x, -halfZ - padEdge }
			};

			int bestSide = 0;
			float bestSideScore = std::numeric_limits<float>::max();
			float bestCorridorIto = minSafeTipY;
			for (int c = 0; c < 4; c++)
			{
				const float cx = candidates[c].x;
				const float cz = candidates[c].z;
				const float dist = std::sqrt((cx - dest.x) * (cx - dest.x) + (cz - dest.z) * (cz - dest.z));
				const int steps = std::clamp(static_cast<int>(std::ceil(dist / 0.75f)), 2, 256);
				float corridorMax = minSafeTipY;
				for (int s = 0; s <= steps; s++)
				{
					const float alpha = static_cast<float>(s) / steps;
					const float sx = std::lerp(cx, dest.x, alpha);
					const float sz = std::lerp(cz, dest.z, alpha);
					// Sample small +/- 3.5 mm neighborhood to stay above wider K16 roughing shoulder cusps
					for (float ox = -3.5f; ox <= 3.5f; ox += 3.5f)
					{
						for (float oz = -3.5f; oz <= 3.5f; oz += 3.5f)
						{
							corridorMax = std::max(corridorMax,
								SampleGridBilinearMax(globalItoGrid, sx + ox, sz + oz, halfX, halfZ, invDx, invDz, divX, divZ));
						}
					}
				}
				const float score = corridorMax * 100.0f + dist;
				if (score < bestSideScore)
				{
					bestSideScore = score;
					bestSide = c;
					bestCorridorIto = corridorMax;
				}
			}

			const float outX = candidates[bestSide].x;
			const float outZ = candidates[bestSide].z;
			const float entryY = std::min(globalModelClearY, std::max({ dest.y + 3.0f, bestCorridorIto + 3.0f }));
			appendSafePoint({ outX, safeY, outZ });
			if (entryY < safeY - 1e-3f)
				appendSafePoint({ outX, entryY, outZ });

			travelY = entryY;
			const float dropY = std::max(0.0f, travelY - dest.y);
			const float rampDist = std::clamp(dropY * 1.25f, 4.0f, 12.0f);

			const float toOutX = outX - dest.x;
			const float toOutZ = outZ - dest.z;
			const float toOutLen = std::sqrt(toOutX * toOutX + toOutZ * toOutZ);
			float bestDirX = (toOutLen > 1e-4f) ? (toOutX / toOutLen) : 1.0f;
			float bestDirZ = (toOutLen > 1e-4f) ? (toOutZ / toOutLen) : 0.0f;
			float bestIto = std::numeric_limits<float>::max();
			constexpr int cNumDirs = 16;
			for (int d = 0; d < cNumDirs; d++)
			{				
				const float ang = (2.0f * std::numbers::pi_v<float> * d) / cNumDirs;
				const float cx = std::cos(ang);
				const float cz = std::sin(ang);
				const float px = dest.x + cx * rampDist;
				const float pz = dest.z + cz * rampDist;
				const float hIto = SampleGridBilinearMax(globalItoGrid, px, pz, halfX, halfZ, invDx, invDz, divX, divZ);
				if (hIto < bestIto - 0.05f || (std::abs(hIto - bestIto) <= 0.05f && (cx * toOutX + cz * toOutZ) > (bestDirX * toOutX + bestDirZ * toOutZ)))
				{
					bestIto = hIto;
					bestDirX = cx;
					bestDirZ = cz;
				}
			}

			const float preX = dest.x + bestDirX * rampDist;
			const float preZ = dest.z + bestDirZ * rampDist;
			appendSafePoint({ preX, travelY, preZ });

			constexpr int cRampChecks = 8;
			for (int s = 1; s < cRampChecks; s++)
			{
				const float alpha = static_cast<float>(s) / cRampChecks;
				const float rx = std::lerp(preX, dest.x, alpha);
				const float rz = std::lerp(preZ, dest.z, alpha);
				const float ry = std::lerp(travelY, dest.y, alpha);
				const float reqY = SampleGridBilinearMax(globalItoGrid, rx, rz, halfX, halfZ, invDx, invDz, divX, divZ);
				if (reqY > ry - 0.15f)
					appendSafePoint({ rx, std::min(travelY, reqY + 0.6f), rz });
			}

			appendSafePoint(dest);
			return;
		}

		{
			const float3 last = waypoints.back();
			const float dxL = dest.x - last.x;
			const float dzL = dest.z - last.z;
			const float distXZ = std::sqrt(dxL * dxL + dzL * dzL);

			// Sample globalItoGrid along the XZ transfer corridor
			const int corridorSteps = std::clamp(static_cast<int>(std::ceil(distXZ / 0.5f)), 2, 256);
			float maxCorridorItoY = minSafeTipY;
			for (int s = 0; s <= corridorSteps; s++)
			{
				const float alpha = static_cast<float>(s) / corridorSteps;
				const float rx = std::lerp(last.x, dest.x, alpha);
				const float rz = std::lerp(last.z, dest.z, alpha);
				maxCorridorItoY = std::max(maxCorridorItoY,
					SampleGridBilinearMax(globalItoGrid, rx, rz, halfX, halfZ, invDx, invDz, divX, divZ));
			}

			// 2.4 mm local margin safely clears 0.5 mm roughing stock + K16 cusps/valleys
			constexpr float cLocalTransferMargin = 2.4f;
			travelY = std::min(globalModelClearY, std::max({ last.y + 0.8f, dest.y + 0.8f, maxCorridorItoY + cLocalTransferMargin }));

			if (last.y < travelY - 1e-3f)
				appendSafePoint({ last.x, travelY, last.z });
		}

		const float dropY = std::max(0.0f, travelY - dest.y);
		const float rampDist = std::clamp(dropY * 0.35f, 1.5f, 6.0f);

		// Pick the safest horizontal approach direction around dest where globalItoGrid is lowest
		float bestDirX = 1.0f;
		float bestDirZ = 0.0f;
		float bestIto = std::numeric_limits<float>::max();
		constexpr int cNumDirs = 16;
		for (int d = 0; d < cNumDirs; d++)
		{
			float ang = (2.0f * 3.14159265f * d) / cNumDirs;
			float cx = std::cos(ang);
			float cz = std::sin(ang);
			float px = dest.x + cx * rampDist;
			float pz = dest.z + cz * rampDist;
			float hIto = SampleGridBilinearMax(globalItoGrid, px, pz, halfX, halfZ, invDx, invDz, divX, divZ);
			if (hIto < bestIto)
			{
				bestIto = hIto;
				bestDirX = cx;
				bestDirZ = cz;
			}
		}

		const float preX = dest.x + bestDirX * rampDist;
		const float preZ = dest.z + bestDirZ * rampDist;
		appendSafePoint({ preX, travelY, preZ });

		// Check intermediate points along the plunge ramp to ensure we never clip any obstacle on the way down
		constexpr int cRampChecks = 8;
		for (int s = 1; s < cRampChecks; s++)
		{
			float alpha = static_cast<float>(s) / cRampChecks;
			float rx = std::lerp(preX, dest.x, alpha);
			float rz = std::lerp(preZ, dest.z, alpha);
			float ry = std::lerp(travelY, dest.y, alpha);
			float reqY = SampleGridBilinearMax(globalItoGrid, rx, rz, halfX, halfZ, invDx, invDz, divX, divZ);
			if (reqY > ry - 0.15f)
			{
				appendSafePoint({ rx, std::min(travelY, reqY + 0.6f), rz });
			}
		}

		appendSafePoint(dest);
	};

	// 6. Generate isoparametric finishing passes for each target surface
	const float scallopH = std::clamp(effectiveParams.scallopHeight, 0.002f, 0.5f);
	const float minStepMm = std::clamp(effectiveParams.minStepover, 0.10f, 5.0f);
	const float maxStepMm = std::clamp(effectiveParams.maxStepover, minStepMm, 6.0f);
	const float curvWeight = std::max(0.0f, effectiveParams.curvatureWeight);
	const float baseFlatWidth = 2.0f * std::sqrt(2.0f * R * scallopH);

	for (size_t surfIdx = 0; surfIdx < numSurf; surfIdx++)
	{
		const auto& surf = modelSurfaces[surfIdx];
		const bool isU = surf.isUDirection;
		const float maxT = isU ? surf.maxU : surf.maxV;
		const float maxW = isU ? surf.maxV : surf.maxU;
		const bool wrapT = isU ? surf.wrapU : surf.wrapV;
		const bool wrapW = isU ? surf.wrapV : surf.wrapU;

		// Build non-self-blocking obstacle grids excluding surfIdx (H_surf^{\neq k} and H_ITO^{\neq k})
		std::vector<float> otherSurfGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());
		std::vector<float> otherItoGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());
		for (size_t m = 0; m < numSurf; m++)
		{
			if (m == surfIdx)
				continue;
			for (size_t cell = 0; cell < otherSurfGrid.size(); cell++)
			{
				otherSurfGrid[cell] = std::max(otherSurfGrid[cell], surfGrids[m][cell]);
				otherItoGrid[cell] = std::max(otherItoGrid[cell], itoGrids[m][cell]);
			}
		}

		auto evalAtTW = [&](float t, float w) -> SurfaceSampleMm
		{
			float u = isU ? t : w;
			float v = isU ? w : t;
			return EvaluateSurfaceSampleMm(surf, u, v, R);
		};

		// Validity predicate for (t, w) on surface k:
		// Checks upper relief visibility, base plane limit, and non-self-blocking C-space ITO collision
		constexpr float cOtherSurfTol = 0.14f;
		constexpr float cOtherItoTol = 0.08f;

		auto isPointValid = [&](float t, float w, SurfaceSampleMm* outSample = nullptr) -> bool
		{
			SurfaceSampleMm s = evalAtTW(t, w);
			if (outSample)
				*outSample = s;

			// 1. Contact point must lie on or above the base plane
			if (s.posMm.y < baseY - 0.02f)
				return false;

			// 2. Outward surface normal must point upward (exclude exact-vertical/undercut closure walls like hull u in [2,3] where n_y == 0,
			//    while keeping steep front flanks with n_y >= 0.005)
			if (s.normal.y < 0.005f)
				return false;

			// 3. Cutter tip must stay above the base plane (15.0 mm)
			if (s.tipMm.y < baseY - 0.01f)
				return false;

			// 4. Cutter tip must lie within the machinable block area
			if (std::abs(s.tipMm.x) > halfX + R + cOutsideBlockMarginMm ||
				std::abs(s.tipMm.z) > halfZ + R + cOutsideBlockMarginMm)
				return false;

			// 5. Contact point must not be submerged inside another surface m != k
			float otherSurfH = SampleGridBilinearMax(
				otherSurfGrid, s.posMm.x, s.posMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
			if (s.posMm.y < otherSurfH - cOtherSurfTol)
				return false;

			// 6. Ball cutter must not gouge any adjacent surface m != k (exact-radius ITO check)
			float otherItoH = SampleGridBilinearMax(
				otherItoGrid, s.tipMm.x, s.tipMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
			if (s.tipMm.y < otherItoH - cOtherItoTol)
				return false;

			return true;
		};

		// Detect true C0 normal discontinuities along cross-parameter w (integer patch boundaries k)
		// under the assumption that pass direction t runs parallel to C0 edges.
		std::vector<float> c0SeamsW;
		const int maxIntW = static_cast<int>(std::floor(maxW + 1e-4f));
		for (int k = 1; k < maxIntW; k++)
		{
			const float fk = static_cast<float>(k);
			if (fk <= 2e-4f || fk >= maxW - 2e-4f)
				continue;
			bool isC0 = false;
			for (int p = 0; p <= 12; p++)
			{
				const float tProbe = (maxT * p) / 12.0f;
				const SurfaceSampleMm sMinus = evalAtTW(tProbe, fk - 1e-4f);
				const SurfaceSampleMm sPlus = evalAtTW(tProbe, fk + 1e-4f);
				if (sMinus.posMm.y < baseY - 0.5f && sPlus.posMm.y < baseY - 0.5f)
					continue;
				if (sMinus.duMm.length_sqr() > 1e-8f && sMinus.dvMm.length_sqr() > 1e-8f &&
					sPlus.duMm.length_sqr() > 1e-8f && sPlus.dvMm.length_sqr() > 1e-8f &&
					Vec3f::dot(sMinus.normal, sPlus.normal) < 0.98f)
				{
					isC0 = true;
					break;
				}
			}
			if (isC0)
				c0SeamsW.push_back(fk);
		}

		auto getPatchBoundsW = [&](float w, float& outMinW, float& outMaxW)
		{
			outMinW = 0.0f;
			outMaxW = maxW;
			for (float seam : c0SeamsW)
			{
				if (w < seam)
				{
					outMaxW = seam - 1e-4f;
					break;
				}
				outMinW = seam + 1e-4f;
			}
		};

		auto crossesC0SeamW = [&](float wA, float wB) -> bool
		{
			const auto [lo, hi] = std::minmax(wA, wB);
			for (float seam : c0SeamsW)
			{
				if (lo < seam && seam < hi)
					return true;
			}
			return false;
		};

		// Estimate adaptive step Delta_w at cross-parameter w using Lecture 4 & 5 curvature-based strip width
		auto computeAdaptiveStepW = [&](float w) -> float
		{
			constexpr int cProbeT = 40;
			const float deltaW = std::max(1e-4f, 0.001f * maxW);
			float patchMinW = 0.0f;
			float patchMaxW = maxW;
			getPatchBoundsW(w, patchMinW, patchMaxW);
			float minParamStepW = maxW;
			bool anyValid = false;

			for (int i = 0; i <= cProbeT; i++)
			{
				float t = (maxT * i) / cProbeT;
				SurfaceSampleMm s0;
				if (!isPointValid(t, w, &s0))
					continue;

				anyValid = true;
				const Vec3f& Pt = isU ? s0.duMm : s0.dvMm;
				const Vec3f& Pw = isU ? s0.dvMm : s0.duMm;
				const float ptLen = Pt.length();
				const float pwLenSq = Pw.length_sqr();
				const float crossLen = Vec3f::cross(Pt, Pw).length();

				// Differential strip width per unit dw perpendicular to pass tangent Pt
				const float lPerp = (ptLen > 1e-4f) ? (crossLen / ptLen) : Pw.length();
				if (lPerp < 1e-3f)
					continue;

				// Transverse normal curvature kappa_perp via finite difference confined to the current smooth w-patch
				const bool useWrapDiff = (wrapW && c0SeamsW.empty());
				float wMinus = useWrapDiff ? WrapOrClampParam(w - deltaW, maxW, true) : std::clamp(w - deltaW, patchMinW, patchMaxW);
				float wPlus = useWrapDiff ? WrapOrClampParam(w + deltaW, maxW, true) : std::clamp(w + deltaW, patchMinW, patchMaxW);
				float actualSpan = (useWrapDiff || (w - deltaW >= patchMinW && w + deltaW <= patchMaxW)) ? (2.0f * deltaW) : (wPlus - wMinus);

				float absKappaPerp = 0.0f;
				float effectiveLPerp = lPerp;
				if (actualSpan > 1e-5f && pwLenSq > 1e-5f)
				{
					SurfaceSampleMm sMinus = evalAtTW(t, wMinus);
					SurfaceSampleMm sPlus = evalAtTW(t, wPlus);
					Vec3f dNormalDw = (sPlus.normal - sMinus.normal) * (1.0f / actualSpan);
					absKappaPerp = std::abs(Vec3f::dot(dNormalDw, Pw)) / pwLenSq;

					// Account for cutter-center arc magnification (1 + R * kappa) on strongly convex ridges
					const Vec3f cMinus = sMinus.posMm + sMinus.normal * R;
					const Vec3f cPlus = sPlus.posMm + sPlus.normal * R;
					const Vec3f dCenterDw = (cPlus - cMinus) * (1.0f / actualSpan);
					const float dCenterPerp = (ptLen > 1e-4f) ? (Vec3f::cross(Pt, dCenterDw).length() / ptLen) : dCenterDw.length();
					effectiveLPerp = std::max(lPerp, 0.65f * dCenterPerp);
				}

				// Strip width formula adjusted for transverse surface curvature
				float denom = std::max(0.20f, 1.0f + curvWeight * R * absKappaPerp);
				float localStripWidthMm = baseFlatWidth / std::sqrt(denom);
				localStripWidthMm = std::clamp(localStripWidthMm, minStepMm, maxStepMm);

				float stepW = localStripWidthMm / effectiveLPerp;
				minParamStepW = std::min(minParamStepW, stepW);
			}

			if (!anyValid)
			{
				// Coarse fallback step when scanning across an unmachined/submerged band of w (or pointed tip w = 0)
				float probeW = std::clamp(w + 0.02f * maxW, patchMinW, patchMaxW);
				SurfaceSampleMm sMid = evalAtTW(0.5f * maxT, probeW);
				const Vec3f& Pw = isU ? sMid.dvMm : sMid.duMm;
				float pwLen = std::max(10.0f, Pw.length());
				return std::clamp(maxStepMm / pwLen, maxW / 400.0f, maxW / 35.0f);
			}

			return std::clamp(minParamStepW, maxW / 500.0f, maxW / 18.0f);
		};

		// Generate sequence of cross-pass parameters w_j in [0, maxW], split cleanly at any C0 seam
		std::vector<float> wValues;
		wValues.reserve(256);
		for (size_t bandIdx = 0; bandIdx <= c0SeamsW.size(); bandIdx++)
		{
			const float bandStartW = (bandIdx == 0) ? 0.0f : (c0SeamsW[bandIdx - 1] + 1e-4f);
			const float bandEndW = (bandIdx < c0SeamsW.size()) ? (c0SeamsW[bandIdx] - 1e-4f) : maxW;
			if (bandEndW <= bandStartW + 1e-5f)
				continue;

			float currW = bandStartW;
			while (currW < bandEndW - 1e-5f)
			{
				wValues.push_back(currW);
				float dw = computeAdaptiveStepW(currW);
				if (currW + dw >= bandEndW)
					break;
				currW += dw;
			}
			const bool isFinalWrappedEnd = (bandIdx == c0SeamsW.size() && wrapW);
			if (!isFinalWrappedEnd)
				wValues.push_back(bandEndW);
		}

		// Extract valid trimmed pass segments for any cross-parameter w
		auto buildPassSegmentsForW = [&](float w) -> std::vector<IsoPassSegment>
		{
			std::vector<float> uvHits = CollectIsoLineUVHits(w, isU, maxT, wrapT, surfaceUVTrimSegments[surfIdx]);

			struct ParamInterval
			{
				float t0 = 0.0f;
				float t1 = 0.0f;
			};
			std::vector<ParamInterval> rawValidIntervals;

			for (size_t r = 0; r + 1 < uvHits.size(); r++)
			{
				const float segT0 = uvHits[r];
				const float segT1 = uvHits[r + 1];
				const float spanT = segT1 - segT0;
				if (spanT < 1e-5f)
					continue;

				// Determine fine sampling count along [segT0, segT1] (~0.18 mm step in 3D)
				SurfaceSampleMm sA = evalAtTW(segT0, w);
				SurfaceSampleMm sB = evalAtTW(0.5f * (segT0 + segT1), w);
				SurfaceSampleMm sC = evalAtTW(segT1, w);
				const float maxDerivMm = std::max({
					(isU ? sA.duMm : sA.dvMm).length(),
					(isU ? sB.duMm : sB.dvMm).length(),
					(isU ? sC.duMm : sC.dvMm).length(),
					1.0f
				});
				const float approxLenMm = spanT * maxDerivMm;
				const int numSteps = std::clamp(static_cast<int>(std::ceil(approxLenMm / 0.18f)), 6, 600);

				bool prevValid = isPointValid(segT0, w);
				float prevT = segT0;
				float activeStartT = prevValid ? segT0 : -1.0f;

				auto bisectBoundary = [&](float tValid, float tInvalid) -> float
				{
					float lo = tValid;
					float hi = tInvalid;
					for (int iter = 0; iter < 12; iter++)
					{
						float mid = 0.5f * (lo + hi);
						if (isPointValid(mid, w))
							lo = mid;
						else
							hi = mid;
					}
					return lo;
				};

				for (int s = 1; s <= numSteps; s++)
				{
					float currT = segT0 + (spanT * s) / numSteps;
					bool currValid = isPointValid(currT, w);

					if (!prevValid && currValid)
					{
						activeStartT = bisectBoundary(currT, prevT);
					}
					else if (prevValid && !currValid)
					{
						float endT = bisectBoundary(prevT, currT);
						if (endT > activeStartT + 1e-5f)
							rawValidIntervals.push_back({ activeStartT, endT });
						activeStartT = -1.0f;
					}

					prevT = currT;
					prevValid = currValid;
				}

				if (prevValid && activeStartT >= 0.0f && segT1 > activeStartT + 1e-5f)
				{
					rawValidIntervals.push_back({ activeStartT, segT1 });
				}
			}

			// Merge consecutive intervals that touch at a UV hit if the boundary point itself is valid and C1-continuous in normal
			std::vector<ParamInterval> mergedIntervals;
			for (const auto& iv : rawValidIntervals)
			{
				bool canMerge = false;
				if (!mergedIntervals.empty() && std::abs(iv.t0 - mergedIntervals.back().t1) < 3e-4f)
				{
					const float tJoin = 0.5f * (iv.t0 + mergedIntervals.back().t1);
					if (isPointValid(tJoin, w))
					{
						SurfaceSampleMm sLeft = evalAtTW(std::max(0.0f, mergedIntervals.back().t1 - 2e-4f), w);
						SurfaceSampleMm sRight = evalAtTW(std::min(maxT, iv.t0 + 2e-4f), w);
						if (Vec3f::dot(sLeft.normal, sRight.normal) >= 0.85f &&
							(sLeft.tipMm - sRight.tipMm).length_sqr() < 0.25f * 0.25f)
						{
							canMerge = true;
						}
					}
				}

				if (canMerge)
					mergedIntervals.back().t1 = iv.t1;
				else
					mergedIntervals.push_back(iv);
			}

			// If the pass parameter t is wrapped and the valid arch crosses the seam t = 0 == maxT,
			// merge the trailing interval [t_a, maxT] with the leading interval [0, t_b] into [t_a, maxT + t_b]
			if (wrapT && mergedIntervals.size() >= 2)
			{
				if (mergedIntervals.front().t0 < 1e-3f && mergedIntervals.back().t1 > maxT - 1e-3f)
				{
					SurfaceSampleMm sZero = evalAtTW(mergedIntervals.front().t0, w);
					SurfaceSampleMm sMax = evalAtTW(mergedIntervals.back().t1, w);
					if ((sZero.tipMm - sMax.tipMm).length_sqr() < 0.05f * 0.05f)
					{
						mergedIntervals.back().t1 = maxT + mergedIntervals.front().t1;
						mergedIntervals.erase(mergedIntervals.begin());
					}
				}
			}

			std::vector<IsoPassSegment> rowSegs;
			for (const auto& iv : mergedIntervals)
			{
				const float spanT = iv.t1 - iv.t0;
				SurfaceSampleMm sStart = evalAtTW(iv.t0, w);
				SurfaceSampleMm sMid = evalAtTW(0.5f * (iv.t0 + iv.t1), w);
				SurfaceSampleMm sEnd = evalAtTW(iv.t1, w);
				const float chordEstimateMm = (sEnd.tipMm - sStart.tipMm).length() + (sMid.tipMm - sStart.tipMm).length();
				if (chordEstimateMm < 0.15f)
					continue;

				const int sampleSteps = std::clamp(static_cast<int>(std::ceil(chordEstimateMm / 0.15f)), 4, 500);
				std::vector<float3> rawPts;
				rawPts.reserve(sampleSteps + 1);

				for (int s = 0; s <= sampleSteps; s++)
				{
					float t = iv.t0 + (spanT * s) / sampleSteps;
					SurfaceSampleMm smp = evalAtTW(t, w);
					float otherItoH = SampleGridBilinearMax(
						otherItoGrid, smp.tipMm.x, smp.tipMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
					float clampedTipY = std::max({ smp.tipMm.y, otherItoH, minSafeTipY });
					rawPts.emplace_back(smp.tipMm.x, clampedTipY, smp.tipMm.z);
				}

				IsoPassSegment seg;
				seg.w = w;
				seg.tStart = iv.t0;
				seg.tEnd = iv.t1;
				seg.pts = SimplifyPolyline3D(rawPts, effectiveParams.simplifyTolerance);
				if (seg.pts.size() >= 2)
					rowSegs.push_back(std::move(seg));
			}
			return rowSegs;
		};

		std::vector<std::vector<IsoPassSegment>> passRows(wValues.size());
		for (size_t j = 0; j < wValues.size(); j++)
			passRows[j] = buildPassSegmentsForW(wValues[j]);

		// Refine cross-parameter boundaries where valid passes terminate against an obstacle ITO,
		// base plane, or normal cliff so no wide strip is left unmilled before the groove/border.
		for (int refPass = 0; refPass < 2; refPass++)
		{
			std::vector<float> nextW;
			std::vector<std::vector<IsoPassSegment>> nextRows;
			nextW.reserve(wValues.size() * 2);
			nextRows.reserve(passRows.size() * 2);

			auto totalSpanT = [](const std::vector<IsoPassSegment>& row) -> float
			{
				float sum = 0.0f;
				for (const auto& s : row) sum += (s.tEnd - s.tStart);
				return sum;
			};

			bool addedAny = false;
			for (size_t j = 0; j < wValues.size(); j++)
			{
				nextW.push_back(wValues[j]);
				nextRows.push_back(std::move(passRows[j]));

				if (j + 1 < wValues.size() && !crossesC0SeamW(wValues[j], wValues[j + 1]))
				{
					const bool emptyA = nextRows.back().empty();
					const bool emptyB = passRows[j + 1].empty();
					const float spanA = totalSpanT(nextRows.back());
					const float spanB = totalSpanT(passRows[j + 1]);
					const bool sharpClip = (!emptyA && !emptyB && std::abs(spanA - spanB) > 0.12f * maxT);
					if ((emptyA != emptyB) || sharpClip)
					{
						const float wMid = 0.5f * (wValues[j] + wValues[j + 1]);
						if (std::abs(wValues[j + 1] - wValues[j]) > maxW / 400.0f)
						{
							auto midSegs = buildPassSegmentsForW(wMid);
							if (!midSegs.empty())
							{
								nextW.push_back(wMid);
								nextRows.push_back(std::move(midSegs));
								addedAny = true;
							}
						}
					}
				}
			}

			wValues = std::move(nextW);
			passRows = std::move(nextRows);
			if (!addedAny)
				break;
		}

		// If the cross-pass parameter w is wrapped and passes are active across w = 0 == maxW,
		// rotate the cyclic pass rows to start right after an empty gap (if any) so the zigzag is unbroken.
		if (wrapW && passRows.size() >= 4 && !passRows.front().empty() && !passRows.back().empty())
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
			{
				std::rotate(passRows.begin(), passRows.begin() + *gapIdx, passRows.end());
			}
		}

		// Group pass segments across consecutive rows into continuous zigzag chains
		struct ZigzagChain
		{
			std::vector<IsoPassSegment> segments;
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

				if (j > 0 && !passRows[j - 1].empty() && !crossesC0SeamW(passRows[j - 1].front().w, row.front().w))
				{
					for (size_t pIdx = 0; pIdx < passRows[j - 1].size(); pIdx++)
					{
						if (prevUsed[pIdx])
							continue;

						const auto& prevSeg = passRows[j - 1][pIdx];
						const float overlapT = std::min(seg.tEnd, prevSeg.tEnd) - std::max(seg.tStart, prevSeg.tStart);
						const float dStart = (segStart - prevSeg.pts.front().ToVec3f()).length();
						const float dEnd = (segEnd - prevSeg.pts.back().ToVec3f()).length();
						const float minEndDist = std::min(dStart, dEnd);
						const float avgEndDist = 0.5f * (dStart + dEnd);

						// Two segments in adjacent w-rows belong to the same continuous patch if their parameter
						// intervals overlap (or nearly touch near an oblique trim curve) and at least one endpoint is reasonably close.
						// Prefer extending longer established chains so small side pockets don't hijack and split the main branch.
						const bool hasParamOverlap = (overlapT > -0.06f * maxT);
						if (hasParamOverlap && minEndDist <= 28.0f && avgEndDist <= 40.0f)
						{
							const float chainBonus = 0.35f * static_cast<float>(chains[prevRowChainIds[pIdx]].segments.size());
							float score = avgEndDist - 6.0f * overlapT - chainBonus;
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

		// Filter out tiny isolated specks (< 2.0 mm total cut length)
		std::vector<ZigzagChain> validChains;
		validChains.reserve(chains.size());
		std::vector<float> chainLengths;
		chainLengths.reserve(chains.size());
		for (auto& c : chains)
		{
			if (c.segments.empty())
				continue;
			float totalLen = 0.0f;
			for (const auto& sg : c.segments)
			{
				for (size_t k = 1; k < sg.pts.size(); k++)
					totalLen += (sg.pts[k].ToVec3f() - sg.pts[k - 1].ToVec3f()).length();
			}
			if (totalLen < 2.0f && c.segments.size() <= 2)
				continue;
			chainLengths.push_back(totalLen);
			validChains.push_back(std::move(c));
		}

		// Visit chains on surface k in branch-coherent greedy nearest-endpoint order,
		// starting with the longest uninterrupted chain so tiny corner pockets are picked up right next to its end.
		std::vector<bool> chainVisited(validChains.size(), false);
		for (size_t visitedCount = 0; visitedCount < validChains.size(); visitedCount++)
		{
			size_t chosenIdx = 0;
			bool reverseRowOrder = false;
			bool initForwardDir = true;

			auto evaluateChainCorners = [&](size_t cIdx, const Vec3f& refPt, bool& outRevRows, bool& outFwd, float& outBestDistSq)
			{
				const auto& segs = validChains[cIdx].segments;
				const float3 corners[4] = {
					segs.front().pts.front(),
					segs.front().pts.back(),
					segs.back().pts.front(),
					segs.back().pts.back()
				};
				outBestDistSq = std::numeric_limits<float>::max();
				for (int corner = 0; corner < 4; corner++)
				{
					float dSq = (corners[corner].ToVec3f() - refPt).length_sqr();
					if (dSq < outBestDistSq)
					{
						outBestDistSq = dSq;
						outRevRows = (corner >= 2);
						outFwd = ((corner % 2) == 0);
					}
				}
			};

			if (visitedCount == 0)
			{
				// Pick the largest main chain on this surface first
				float maxLen = -1.0f;
				for (size_t cIdx = 0; cIdx < validChains.size(); cIdx++)
				{
					if (chainLengths[cIdx] > maxLen)
					{
						maxLen = chainLengths[cIdx];
						chosenIdx = cIdx;
					}
				}

				const auto& segs = validChains[chosenIdx].segments;
				if (!waypoints.empty())
				{
					float dummyDistSq = 0.0f;
					evaluateChainCorners(chosenIdx, waypoints.back().ToVec3f(), reverseRowOrder, initForwardDir, dummyDistSq);
				}
				else
				{
					// Start from the higher-Y end of the main chain so we finish down near any lower split pockets
					float frontAvgY = 0.5f * (segs.front().pts.front().y + segs.front().pts.back().y);
					float backAvgY = 0.5f * (segs.back().pts.front().y + segs.back().pts.back().y);
					reverseRowOrder = (backAvgY > frontAvgY);
					const auto& firstPts = reverseRowOrder ? segs.back().pts : segs.front().pts;
					initForwardDir = (firstPts.back().y - firstPts.front().y >= -1.0f);
				}
			}
			else
			{
				const Vec3f refPt = waypoints.back().ToVec3f();
				float bestDistSq = std::numeric_limits<float>::max();
				for (size_t cIdx = 0; cIdx < validChains.size(); cIdx++)
				{
					if (chainVisited[cIdx])
						continue;
					bool rev = false;
					bool fwd = true;
					float dSq = 0.0f;
					evaluateChainCorners(cIdx, refPt, rev, fwd, dSq);
					if (dSq < bestDistSq)
					{
						bestDistSq = dSq;
						chosenIdx = cIdx;
						reverseRowOrder = rev;
						initForwardDir = fwd;
					}
				}
			}

			chainVisited[chosenIdx] = true;
			auto& chain = validChains[chosenIdx];
			if (reverseRowOrder)
				std::reverse(chain.segments.begin(), chain.segments.end());

			bool forwardDir = initForwardDir;
			float actualPrevEndT = 0.0f;
			float actualPrevW = 0.0f;

			for (size_t sIdx = 0; sIdx < chain.segments.size(); sIdx++)
			{
				const auto& seg = chain.segments[sIdx];
				std::vector<float3> segPts = seg.pts;
				if (!forwardDir)
					std::reverse(segPts.begin(), segPts.end());

				const float actualCurrStartT = forwardDir ? seg.tStart : seg.tEnd;
				const float actualCurrEndT = forwardDir ? seg.tEnd : seg.tStart;
				const float actualCurrW = seg.w;

				if (sIdx == 0)
				{
					const Vec3f nextStart = segPts.front().ToVec3f();
					if (!waypoints.empty() &&
						(nextStart - waypoints.back().ToVec3f()).length() <= 4.0f &&
						isStraightSegmentCollisionFree(waypoints.back().ToVec3f(), nextStart, otherItoGrid, 0.14f))
					{
						appendSafePoint(segPts.front());
					}
					else
					{
						transitionSafeCl(segPts.front());
					}
				}
				else
				{
					const Vec3f prevEnd = waypoints.back().ToVec3f();
					const Vec3f nextStart = segPts.front().ToVec3f();
					const float linkDist = (nextStart - prevEnd).length();

					bool straightLineSafe = (linkDist <= 28.0f) &&
						isStraightSegmentCollisionFree(prevEnd, nextStart, otherItoGrid, 0.14f);
					if (straightLineSafe && linkDist > 3.5f)
					{
						straightLineSafe = isStraightSegmentCollisionFree(prevEnd, nextStart, itoGrids[surfIdx], 0.45f);
					}

					// When linking across a trim boundary (> 0.70 mm gap between pass end-caps),
					// prefer hugging the trimmed surface/ITO shoulder so diagonal or arched chords don't leave flutes.
					const bool canTryBoundaryTrace =
						(linkDist > 0.70f && linkDist <= 28.0f) &&
						std::abs(actualCurrStartT - actualPrevEndT) < 0.45f * maxT &&
						std::abs(actualCurrW - actualPrevW) < 0.25f * maxW;

					if (linkDist <= 0.70f && straightLineSafe)
					{
						appendSafePoint(segPts.front());
					}
					else if (canTryBoundaryTrace)
					{
						bool boundaryTraceSafe = true;
						std::vector<float3> linkPts;
						const int linkSteps = std::clamp(static_cast<int>(std::ceil(linkDist / 0.50f)), 2, 48);
						for (int ls = 1; ls < linkSteps; ls++)
						{
							const float alpha = static_cast<float>(ls) / linkSteps;
							const float tInterp = std::lerp(actualPrevEndT, actualCurrStartT, alpha);
							const float wInterp = std::lerp(actualPrevW, actualCurrW, alpha);
							SurfaceSampleMm smp = evalAtTW(tInterp, wInterp);
							const float reqOtherIto = SampleGridBilinearMax(
								otherItoGrid, smp.tipMm.x, smp.tipMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
							const float safeLinkY = std::max({ smp.tipMm.y, reqOtherIto, minSafeTipY });
							if (safeLinkY - std::max(prevEnd.y, nextStart.y) > 4.0f)
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
							transitionSafeCl(segPts.front());
						}
					}
					else if (straightLineSafe)
					{
						appendSafePoint(segPts.front());
					}
					else
					{
						transitionSafeCl(segPts.front());
					}
				}

				for (size_t k = 1; k < segPts.size(); k++)
					appendSafePoint(segPts[k]);

				actualPrevEndT = actualCurrEndT;
				actualPrevW = actualCurrW;
				forwardDir = !forwardDir;
			}
		}
	}

	// 7. Pencil Milling / Intersection Groove Passes (Lecture 8 bi-tangent ball center formula)
	if (effectiveParams.includeGroovePasses && !grooveCurves.empty())
	{
		std::vector<std::vector<float3>> activeGrooveChains;

		for (const auto& groove : grooveCurves)
		{
			const size_t idxA = groove.idxA;
			const size_t idxB = groove.idxB;
			const auto& surfA = modelSurfaces[idxA];
			const auto& surfB = modelSurfaces[idxB];

			// Build third-party obstacle grids excluding {idxA, idxB} so neither intersecting surface blocks the groove pass
			std::vector<float> thirdSurfGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());
			std::vector<float> thirdItoGrid(static_cast<size_t>(cols) * rows, -std::numeric_limits<float>::max());
			for (size_t m = 0; m < numSurf; m++)
			{
				if (m == idxA || m == idxB)
					continue;
				for (size_t cell = 0; cell < thirdSurfGrid.size(); cell++)
				{
					thirdSurfGrid[cell] = std::max(thirdSurfGrid[cell], surfGrids[m][cell]);
					thirdItoGrid[cell] = std::max(thirdItoGrid[cell], itoGrids[m][cell]);
				}
			}

			std::vector<float3> currentChain;
			const size_t numPts = groove.data.points.size();

			auto flushCurrentGrooveChain = [&]()
			{
				if (currentChain.size() >= 2)
				{
					// Smooth out 0.20 mm ITO-grid quantization jitter from the bi-tangent valley search
					if (currentChain.size() >= 4)
					{
						for (int pass = 0; pass < 10; pass++)
						{
							std::vector<float3> tmp = currentChain;
							for (size_t k = 1; k + 1 < currentChain.size(); k++)
							{
								tmp[k].x = 0.25f * currentChain[k - 1].x + 0.5f * currentChain[k].x + 0.25f * currentChain[k + 1].x;
								tmp[k].z = 0.25f * currentChain[k - 1].z + 0.5f * currentChain[k].z + 0.25f * currentChain[k + 1].z;
								if (pass < 4)
								{
									const float smoothY = 0.25f * currentChain[k - 1].y + 0.5f * currentChain[k].y + 0.25f * currentChain[k + 1].y;
									tmp[k].y = std::max(currentChain[k].y, smoothY);
								}
							}
							currentChain.swap(tmp);
						}
					}

					float chainLen = 0.0f;
					for (size_t k = 1; k < currentChain.size(); k++)
						chainLen += (currentChain[k].ToVec3f() - currentChain[k - 1].ToVec3f()).length();
					if (chainLen >= 1.5f)
						activeGrooveChains.push_back(SimplifyPolyline3D(currentChain, std::max(effectiveParams.simplifyTolerance, 0.025f)));
				}
				currentChain.clear();
			};

			for (size_t i = 0; i < numPts; i++)
			{
				const Vec3f p0Mm = groove.data.points[i] * cWorldToMm;
				const Vec4f uv = groove.data.params[i];

				if (p0Mm.y < baseY - 0.05f)
				{
					flushCurrentGrooveChain();
					continue;
				}

				// Check if the intersection curve point P0 is buried inside a third surface m not in {A, B}
				float thirdSurfH = SampleGridBilinearMax(
					thirdSurfGrid, p0Mm.x, p0Mm.z, halfX, halfZ, invDx, invDz, divX, divZ);
				if (p0Mm.y < thirdSurfH - 0.18f)
				{
					flushCurrentGrooveChain();
					continue;
				}

				SurfaceSampleMm smpA = EvaluateSurfaceSampleMm(surfA, uv.x, uv.y, R);
				SurfaceSampleMm smpB = EvaluateSurfaceSampleMm(surfB, uv.z, uv.w, R);
				Vec3f nA = smpA.normal;
				Vec3f nB = smpB.normal;

				// Ensure neither surface normal points steeply into the underside of the relief
				if (nA.y < -0.02f || nB.y < -0.02f)
				{
					flushCurrentGrooveChain();
					continue;
				}

				float dotAB = std::clamp(Vec3f::dot(nA, nB), -0.92f, 0.999f);
				Vec3f ballCenter = p0Mm + (nA + nB) * (R / (1.0f + dotAB));
				Vec3f tipMm = ballCenter - Vec3f(0.0f, R, 0.0f);

				const bool hasVerticalWall = (std::min(nA.y, nB.y) < 0.15f);
				if (hasVerticalWall)
				{
					const bool wallIsB = (nB.y <= nA.y);
					const size_t floorIdx = wallIsB ? idxA : idxB;
					const size_t wallIdx = wallIsB ? idxB : idxA;
					const Vec3f& nWall = wallIsB ? nB : nA;
					const Vec3f& nFloor = wallIsB ? nA : nB;

					float wx = (nFloor.y >= 0.08f) ? nWall.x : (nA.x + nB.x);
					float wz = (nFloor.y >= 0.08f) ? nWall.z : (nA.z + nB.z);
					const float wLen = std::sqrt(wx * wx + wz * wz);
					if (wLen > 1e-4f)
					{
						wx /= wLen;
						wz /= wLen;
					}
					else
					{
						wx = 0.0f;
						wz = -1.0f;
					}

					tipMm.x = std::clamp(tipMm.x + 0.04f * wx, -halfX, halfX);
					tipMm.z = std::clamp(tipMm.z + 0.04f * wz, -halfZ, halfZ);

					// Guard against any overhang on the vertical wall leaning into the cutter above P0
					const float checkR = std::max(0.5f, R - 0.14f);
					const float checkR2 = checkR * checkR;
					for (int backStep = 0; backStep < 12; backStep++)
					{
						bool wallOverhangHit = false;
						const int gxMin = std::max(0, static_cast<int>(std::floor((tipMm.x - checkR + halfX) * invDx)));
						const int gxMax = std::min(divX, static_cast<int>(std::ceil((tipMm.x + checkR + halfX) * invDx)));
						const int gzMin = std::max(0, static_cast<int>(std::floor((tipMm.z - checkR + halfZ) * invDz)));
						const int gzMax = std::min(divZ, static_cast<int>(std::ceil((tipMm.z + checkR + halfZ) * invDz)));
						for (int gz = gzMin; gz <= gzMax && !wallOverhangHit; gz++)
						{
							const float cellZ = -halfZ + gz * dz;
							const float dcz = cellZ - tipMm.z;
							for (int gx = gxMin; gx <= gxMax; gx++)
							{
								const float cellX = -halfX + gx * dx;
								const float dcx = cellX - tipMm.x;
								const float r2 = dcx * dcx + dcz * dcz;
								if (r2 >= checkR2)
									continue;
								const size_t cell = static_cast<size_t>(gz) * cols + gx;
								const float wallH = (nFloor.y >= 0.08f)
									? surfGrids[wallIdx][cell]
									: std::max(surfGrids[idxA][cell], surfGrids[idxB][cell]);
								const float ballBottomH = tipMm.y + R - std::sqrt(std::max(0.0f, R2 - r2));
								if (wallH > ballBottomH + 0.04f)
								{
									wallOverhangHit = true;
									break;
								}
							}
						}
						if (!wallOverhangHit)
							break;
						tipMm.x = std::clamp(tipMm.x + 0.05f * wx, -halfX, halfX);
						tipMm.z = std::clamp(tipMm.z + 0.05f * wz, -halfZ, halfZ);
					}

					if (nFloor.y >= 0.08f)
					{
						const float itoFloor = SampleGridBilinearMax(
							itoGrids[floorIdx], tipMm.x, tipMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
						tipMm.y = std::max({ tipMm.y, itoFloor + 0.02f, minSafeTipY });
					}
					else
					{
						tipMm.y = std::max(tipMm.y, minSafeTipY);
					}
				}
				else
				{
					// Clamp horizontal seed offset from P0 to R so strongly convex walls don't overshoot
					float latX = tipMm.x - p0Mm.x;
					float latZ = tipMm.z - p0Mm.z;
					float latLen = std::sqrt(latX * latX + latZ * latZ);
					if (latLen > R)
					{
						latX *= (R / latLen);
						latZ *= (R / latLen);
					}
					const float seedX = std::clamp(p0Mm.x + latX, -halfX, halfX);
					const float seedZ = std::clamp(p0Mm.z + latZ, -halfZ, halfZ);

					// Cross-section direction perpendicular to intersection curve tangent in XZ
					const size_t iPrev = (i > 0) ? (i - 1) : 0;
					const size_t iNext = (i + 1 < numPts) ? (i + 1) : (numPts - 1);
					const Vec3f pPrev = groove.data.points[iPrev] * cWorldToMm;
					const Vec3f pNext = groove.data.points[iNext] * cWorldToMm;
					float tanX = pNext.x - pPrev.x;
					float tanZ = pNext.z - pPrev.z;
					float tanLen = std::sqrt(tanX * tanX + tanZ * tanZ);
					float perpX = 0.0f;
					float perpZ = 1.0f;
					if (tanLen > 1e-4f)
					{
						perpX = -tanZ / tanLen;
						perpZ = tanX / tanLen;
					}
					else if (latLen > 1e-4f)
					{
						perpX = latX / latLen;
						perpZ = latZ / latLen;
					}

					// Locate exact C-space bi-tangent valley argmin_u max(itoA, itoB) across the groove
					// so convex tubes (e.g. wave1/wave2) seat fully into the crease without floating high.
					float bestU = 0.0f;
					float bestX = seedX;
					float bestZ = seedZ;
					float ia0 = SampleGridBilinearMax(itoGrids[idxA], seedX, seedZ, halfX, halfZ, invDx, invDz, divX, divZ);
					float ib0 = SampleGridBilinearMax(itoGrids[idxB], seedX, seedZ, halfX, halfZ, invDx, invDz, divX, divZ);
					float bestMaxAB = std::max(ia0, ib0);
					float bestScore = bestMaxAB + 0.55f * std::abs(ia0 - ib0);

					constexpr float cSearchHalfWidth = 2.2f;
					for (float u = -cSearchHalfWidth; u <= cSearchHalfWidth + 1e-4f; u += 0.14f)
					{
						const float cx = std::clamp(seedX + u * perpX, -halfX, halfX);
						const float cz = std::clamp(seedZ + u * perpZ, -halfZ, halfZ);
						const float ia = SampleGridBilinearMax(itoGrids[idxA], cx, cz, halfX, halfZ, invDx, invDz, divX, divZ);
						const float ib = SampleGridBilinearMax(itoGrids[idxB], cx, cz, halfX, halfZ, invDx, invDz, divX, divZ);
						const float valY = std::max(ia, ib);
						// Lock onto the bi-tangent crease (itoA == itoB) rather than sliding down one open flank
						const float score = valY + 0.55f * std::abs(ia - ib) + 0.025f * std::abs(u);
						if (score < bestScore)
						{
							bestScore = score;
							bestMaxAB = valY;
							bestU = u;
							bestX = cx;
							bestZ = cz;
						}
					}

					for (float du = -0.14f; du <= 0.14f + 1e-4f; du += 0.035f)
					{
						const float u = bestU + du;
						const float cx = std::clamp(seedX + u * perpX, -halfX, halfX);
						const float cz = std::clamp(seedZ + u * perpZ, -halfZ, halfZ);
						const float ia = SampleGridBilinearMax(itoGrids[idxA], cx, cz, halfX, halfZ, invDx, invDz, divX, divZ);
						const float ib = SampleGridBilinearMax(itoGrids[idxB], cx, cz, halfX, halfZ, invDx, invDz, divX, divZ);
						const float valY = std::max(ia, ib);
						// Lock onto the bi-tangent crease (itoA == itoB) rather than sliding down one open flank
						const float score = valY + 0.55f * std::abs(ia - ib) + 0.025f * std::abs(u);
						if (score < bestScore)
						{
							bestScore = score;
							bestMaxAB = valY;
							bestX = cx;
							bestZ = cz;
						}
					}

					tipMm.x = bestX;
					tipMm.z = bestZ;
					tipMm.y = std::max(bestMaxAB + 0.02f, minSafeTipY);
				}

				// Check if a third surface m not in {A, B} blocks the ball cutter at this groove location
				float thirdItoH = SampleGridBilinearMax(
					thirdItoGrid, tipMm.x, tipMm.z, halfX, halfZ, invDx, invDz, divX, divZ);
				if (tipMm.y < thirdItoH - 0.12f)
				{
					flushCurrentGrooveChain();
					continue;
				}

				// Reject groove waypoints where the ball cutter floats in thin air (> R + 0.25 mm from A and B)
				{
					const float bcY = tipMm.y + R;
					const float maxTouchR = R + 0.25f;
					const float maxTouchR2 = maxTouchR * maxTouchR;
					bool touchesModel = ((Vec3f(tipMm.x, bcY, tipMm.z) - p0Mm).length_sqr() <= maxTouchR2);
					if (!touchesModel)
					{
						const int gxMin = std::max(0, static_cast<int>(std::floor((tipMm.x - maxTouchR + halfX) * invDx)));
						const int gxMax = std::min(divX, static_cast<int>(std::ceil((tipMm.x + maxTouchR + halfX) * invDx)));
						const int gzMin = std::max(0, static_cast<int>(std::floor((tipMm.z - maxTouchR + halfZ) * invDz)));
						const int gzMax = std::min(divZ, static_cast<int>(std::ceil((tipMm.z + maxTouchR + halfZ) * invDz)));
						for (int gz = gzMin; gz <= gzMax && !touchesModel; gz++)
						{
							const float dcz = (-halfZ + gz * dz) - tipMm.z;
							for (int gx = gxMin; gx <= gxMax; gx++)
							{
								const float dcx = (-halfX + gx * dx) - tipMm.x;
								const float xz2 = dcx * dcx + dcz * dcz;
								if (xz2 > maxTouchR2)
									continue;
								const size_t cell = static_cast<size_t>(gz) * cols + gx;
								const float hA = surfGrids[idxA][cell];
								if (hA > -1e20f)
								{
									const float dy = hA - bcY;
									if (xz2 + dy * dy <= maxTouchR2) { touchesModel = true; break; }
								}
								const float hB = surfGrids[idxB][cell];
								if (hB > -1e20f)
								{
									const float dy = hB - bcY;
									if (xz2 + dy * dy <= maxTouchR2) { touchesModel = true; break; }
								}
							}
						}
					}
					if (!touchesModel)
					{
						flushCurrentGrooveChain();
						continue;
					}
				}

				currentChain.emplace_back(tipMm.x, tipMm.y, tipMm.z);
			}

			flushCurrentGrooveChain();
		}

		// Connect active groove chains in greedy nearest-neighbor order to minimize air travel
		std::vector<bool> grooveUsed(activeGrooveChains.size(), false);
		for (size_t step = 0; step < activeGrooveChains.size(); step++)
		{
			Vec3f refPos = waypoints.empty() ? Vec3f(0.0f, safeY, 0.0f) : waypoints.back().ToVec3f();
			float bestDistSq = std::numeric_limits<float>::max();
			std::optional<size_t> bestIdx;
			bool reverseBest = false;

			for (size_t cIdx = 0; cIdx < activeGrooveChains.size(); cIdx++)
			{
				if (grooveUsed[cIdx] || activeGrooveChains[cIdx].size() < 2)
					continue;

				const auto& ch = activeGrooveChains[cIdx];
				float dFrontSq = (ch.front().ToVec3f() - refPos).length_sqr();
				float dBackSq = (ch.back().ToVec3f() - refPos).length_sqr();

				if (dFrontSq < bestDistSq)
				{
					bestDistSq = dFrontSq;
					bestIdx = cIdx;
					reverseBest = false;
				}
				if (dBackSq < bestDistSq)
				{
					bestDistSq = dBackSq;
					bestIdx = cIdx;
					reverseBest = true;
				}
			}

			if (!bestIdx)
				break;

			grooveUsed[*bestIdx] = true;
			std::vector<float3> ch = activeGrooveChains[*bestIdx];
			if (reverseBest)
				std::reverse(ch.begin(), ch.end());

			// Orient steep groove segments bottom-up if needed
			{
				const Vec3f p0 = ch.front().ToVec3f();
				const Vec3f p1 = ch.back().ToVec3f();
				const float dy = p1.y - p0.y;
				const float lenXZ = std::sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.z - p0.z) * (p1.z - p0.z));
				if (dy < -0.5f && lenXZ < (-dy) * cMinSafeHorizontalSlope * 1.25f)
					std::reverse(ch.begin(), ch.end());
			}

			const float grooveGap = (ch.front().ToVec3f() - waypoints.back().ToVec3f()).length();
			const float grooveTol = (grooveGap <= 1.2f) ? 0.16f : 0.08f;
			if (!waypoints.empty() &&
				grooveGap <= 3.5f &&
				isStraightSegmentCollisionFree(waypoints.back().ToVec3f(), ch.front().ToVec3f(), globalItoGrid, grooveTol))
			{
				appendSafePoint(ch.front());
			}
			else
			{
				transitionSafeCl(ch.front());
			}

			for (size_t k = 1; k < ch.size(); k++)
				appendSafePoint(ch[k]);
		}
	}

	// 7b. Clear enclosed flat base pockets unreachable from the block exterior in Stage 2 (e.g. between wave1 & wave2)
	{
		const int gridW = divX + 1;
		const int gridH = divZ + 1;
		const size_t totalCells = static_cast<size_t>(gridW) * gridH;
		const float flatClearTol = 0.35f;

		auto isBaseReachable = [&](size_t idx) -> bool
		{
			return globalItoGrid[idx] <= baseY + flatClearTol;
		};

		std::vector<uint8_t> exteriorVisited(totalCells, 0);
		std::vector<int> bfsQueue;
		bfsQueue.reserve(totalCells / 4);

		auto pushExterior = [&](int ix, int iz)
		{
			const size_t idx = static_cast<size_t>(iz) * gridW + ix;
			if (!exteriorVisited[idx] && isBaseReachable(idx))
			{
				exteriorVisited[idx] = 1;
				bfsQueue.push_back(static_cast<int>(idx));
			}
		};

		for (int ix = 0; ix <= divX; ix++)
		{
			pushExterior(ix, 0);
			pushExterior(ix, divZ);
		}
		for (int iz = 1; iz < divZ; iz++)
		{
			pushExterior(0, iz);
			pushExterior(divX, iz);
		}

		for (size_t head = 0; head < bfsQueue.size(); head++)
		{
			const int cur = bfsQueue[head];
			const int cx = cur % gridW;
			const int cz = cur / gridW;
			const int ddx[4] = { -1, 1, 0, 0 };
			const int ddz[4] = { 0, 0, -1, 1 };
			for (int k = 0; k < 4; k++)
			{
				const int nx = cx + ddx[k];
				const int nz = cz + ddz[k];
				if (nx >= 0 && nx <= divX && nz >= 0 && nz <= divZ)
					pushExterior(nx, nz);
			}
		}

		// Group unvisited reachable base cells into 4-connected pocket components
		std::vector<uint8_t> compAssigned(totalCells, 0);
		struct PocketRun { int iz; int ix0; int ix1; bool used = false; };
		const float flatStep = std::clamp(
			2.0f * std::sqrt(std::max(0.01f, 2.0f * R * params.scallopHeight - params.scallopHeight * params.scallopHeight)),
			params.minStepover,
			params.maxStepover);
		const int stepIz = std::max(1, static_cast<int>(std::round(flatStep / dz)));
		const float entryAirY = std::max(params.blockSizeY + 1.0f, 51.0f);

		for (int startIz = 1; startIz < divZ; startIz++)
		{
			for (int startIx = 1; startIx < divX; startIx++)
			{
				const size_t startIdx = static_cast<size_t>(startIz) * gridW + startIx;
				if (exteriorVisited[startIdx] || compAssigned[startIdx] || !isBaseReachable(startIdx))
					continue;

				std::vector<int> compCells;
				compAssigned[startIdx] = 1;
				compCells.push_back(static_cast<int>(startIdx));
				int minIz = startIz, maxIz = startIz;

				for (size_t head = 0; head < compCells.size(); head++)
				{
					const int cur = compCells[head];
					const int cx = cur % gridW;
					const int cz = cur / gridW;
					minIz = std::min(minIz, cz);
					maxIz = std::max(maxIz, cz);
					const int ddx[4] = { -1, 1, 0, 0 };
					const int ddz[4] = { 0, 0, -1, 1 };
					for (int k = 0; k < 4; k++)
					{
						const int nx = cx + ddx[k];
						const int nz = cz + ddz[k];
						if (nx >= 0 && nx <= divX && nz >= 0 && nz <= divZ)
						{
							const size_t nIdx = static_cast<size_t>(nz) * gridW + nx;
							if (!exteriorVisited[nIdx] && !compAssigned[nIdx] && isBaseReachable(nIdx))
							{
								compAssigned[nIdx] = 1;
								compCells.push_back(static_cast<int>(nIdx));
							}
						}
					}
				}

				if (compCells.size() < 20)
					continue;

				// Build per-row mask for this component only
				std::vector<uint8_t> inComp(totalCells, 0);
				for (int idx : compCells)
					inComp[idx] = 1;

				std::vector<PocketRun> runs;
				for (int iz = minIz; iz <= maxIz; iz++)
				{
					const bool isSampleRow = (iz == minIz || iz == maxIz || ((iz - minIz) % stepIz == 0));
					if (!isSampleRow)
						continue;

					int ix = 0;
					while (ix <= divX)
					{
						while (ix <= divX && !inComp[static_cast<size_t>(iz) * gridW + ix])
							ix++;
						if (ix > divX)
							break;
						int ix0 = ix;
						while (ix <= divX && inComp[static_cast<size_t>(iz) * gridW + ix])
							ix++;
						int ix1 = ix - 1;
						if (ix1 > ix0)
							runs.push_back({ iz, ix0, ix1, false });
					}
				}

				if (runs.empty())
					continue;

				auto getFloorY = [&](float x, float z) -> float
				{
					const float ito = SampleGridBilinearMax(
						globalItoGrid, x, z, halfX, halfZ, invDx, invDz, divX, divZ);
					return std::max(baseY, ito + 0.02f);
				};

				auto traceFloorSegment = [&](float x0, float x1, float zp)
				{
					const float len = std::abs(x1 - x0);
					const int segSteps = std::max(1, static_cast<int>(std::ceil(len / 0.5f)));
					for (int s = 0; s <= segSteps; s++)
					{
						const float t = static_cast<float>(s) / segSteps;
						const float px = std::lerp(x0, x1, t);
						appendSafePoint(float3(px, getFloorY(px, zp), zp));
					}
				};

				auto rampAndTraceRun = [&](const PocketRun& r, bool startAtLeft)
				{
					const float xA = -halfX + r.ix0 * dx;
					const float xB = -halfX + r.ix1 * dx;
					const float zp = -halfZ + r.iz * dz;
					const float runLen = std::max(0.1f, xB - xA);
					const float xStart = startAtLeft ? xA : xB;
					const float xEnd = startAtLeft ? xB : xA;
					const float yStart = getFloorY(xStart, zp);
					const float yEnd = getFloorY(xEnd, zp);

					const Vec3f startFloor(xStart, yStart, zp);
					if (!waypoints.empty() &&
						std::abs(waypoints.back().y - yStart) <= 0.45f &&
						isStraightSegmentCollisionFree(waypoints.back().ToVec3f(), startFloor, globalItoGrid, 0.05f))
					{
						traceFloorSegment(xStart, xEnd, zp);
						return;
					}

					// Safe angled ramp entry from above stock down to pocket floor
					transitionSafeCl(float3(xStart, entryAirY, zp));
					if (runLen >= 0.4f)
					{
						const float dyMax = std::clamp(runLen * 0.5f, 0.25f, 8.0f);
						float curY = entryAirY;
						bool atStart = true;
						const float floorTargetY = std::max(yStart, yEnd);
						while (curY > floorTargetY + 1e-4f)
						{
							curY = std::max(floorTargetY, curY - dyMax);
							atStart = !atStart;
							const float tx = atStart ? xStart : xEnd;
							appendSafePoint(float3(tx, std::max(curY, getFloorY(tx, zp)), zp));
						}
						if (atStart)
							traceFloorSegment(xStart, xEnd, zp);
						else
							traceFloorSegment(xEnd, xStart, zp);
					}
					else
					{
						traceFloorSegment(xStart, xEnd, zp);
					}
				};

				// Visit runs in continuous collision-free zigzag order, ramping cleanly into any disconnected sub-lobes
				for (size_t done = 0; done < runs.size(); done++)
				{
					const Vec3f curPos = waypoints.empty() ? Vec3f(0.0f, entryAirY, 0.0f) : waypoints.back().ToVec3f();
					size_t bestIdx = 0;
					bool bestStartLeft = true;
					bool bestDirect = false;
					float bestDistSq = std::numeric_limits<float>::max();

					for (size_t rIdx = 0; rIdx < runs.size(); rIdx++)
					{
						if (runs[rIdx].used)
							continue;
						const float zp = -halfZ + runs[rIdx].iz * dz;
						const float xL = -halfX + runs[rIdx].ix0 * dx;
						const float xR = -halfX + runs[rIdx].ix1 * dx;
						const Vec3f pLeft(xL, getFloorY(xL, zp), zp);
						const Vec3f pRight(xR, getFloorY(xR, zp), zp);

						for (int side = 0; side < 2; side++)
						{
							const Vec3f& candidate = (side == 0) ? pLeft : pRight;
							const bool direct = !waypoints.empty() &&
								std::abs(curPos.y - candidate.y) <= 0.45f &&
								isStraightSegmentCollisionFree(curPos, candidate, globalItoGrid, 0.05f);
							const float dSq = (candidate - curPos).length_sqr();

							if ((direct && !bestDirect) || (direct == bestDirect && dSq < bestDistSq))
							{
								bestDirect = direct;
								bestDistSq = dSq;
								bestIdx = rIdx;
								bestStartLeft = (side == 0);
							}
						}
					}

					runs[bestIdx].used = true;
					rampAndTraceRun(runs[bestIdx], bestStartLeft);
				}

				// Contour pass around the smoothed pocket boundary to remove raster staircase cusps along wave bases
				{
					const int vW = gridW + 1;
					const int vH = gridH + 1;
					std::vector<uint8_t> edgeMask(static_cast<size_t>(vW) * vH, 0);
					const int edx[4] = { 1, 0, -1, 0 };
					const int edz[4] = { 0, 1, 0, -1 };

					for (int idx : compCells)
					{
						const int cx = idx % gridW;
						const int cz = idx / gridW;
						if (cz == 0 || !inComp[static_cast<size_t>(cz - 1) * gridW + cx])
							edgeMask[static_cast<size_t>(cz) * vW + cx] |= (1u << 0);
						if (cx == divX || !inComp[static_cast<size_t>(cz) * gridW + (cx + 1)])
							edgeMask[static_cast<size_t>(cz) * vW + (cx + 1)] |= (1u << 1);
						if (cz == divZ || !inComp[static_cast<size_t>(cz + 1) * gridW + cx])
							edgeMask[static_cast<size_t>(cz + 1) * vW + (cx + 1)] |= (1u << 2);
						if (cx == 0 || !inComp[static_cast<size_t>(cz) * gridW + (cx - 1)])
							edgeMask[static_cast<size_t>(cz + 1) * vW + cx] |= (1u << 3);
					}

					for (int svz = 0; svz < vH; svz++)
					{
						for (int svx = 0; svx < vW; svx++)
						{
							while (edgeMask[static_cast<size_t>(svz) * vW + svx] != 0)
							{
								int startDir = 0;
								while (startDir < 4 && !(edgeMask[static_cast<size_t>(svz) * vW + svx] & (1u << startDir)))
									startDir++;

								std::vector<Vec2f> rawLoop;
								int vx = svx;
								int vz = svz;
								int dir = startDir;

								while (true)
								{
									const size_t vIdx = static_cast<size_t>(vz) * vW + vx;
									if (!(edgeMask[vIdx] & (1u << dir)))
										break;
									edgeMask[vIdx] &= static_cast<uint8_t>(~(1u << dir));

									// Sub-cell iso-level crossing on globalItoGrid along the cell-to-cell segment
									const float thresh = baseY + flatClearTol;
									auto sampleIto = [&](int ix, int iz) -> float
									{
										if (ix < 0 || ix > divX || iz < 0 || iz > divZ)
											return thresh + 0.5f;
										return globalItoGrid[static_cast<size_t>(iz) * gridW + ix];
									};
									int inX = vx, inZ = vz, outX = vx, outZ = vz;
									if (dir == 0) { inX = vx; inZ = vz; outX = vx; outZ = vz - 1; }
									else if (dir == 1) { inX = vx - 1; inZ = vz; outX = vx; outZ = vz; }
									else if (dir == 2) { inX = vx - 1; inZ = vz - 1; outX = vx - 1; outZ = vz; }
									else { inX = vx; inZ = vz - 1; outX = vx - 1; outZ = vz - 1; }
									const float gIn = sampleIto(inX, inZ);
									const float gOut = sampleIto(outX, outZ);
									const float frac = (gOut > gIn + 1e-4f)
										? std::clamp((thresh - gIn) / (gOut - gIn), 0.0f, 1.0f)
										: 0.5f;
									const float crossX = -halfX + std::lerp(static_cast<float>(inX), static_cast<float>(outX), frac) * dx;
									const float crossZ = -halfZ + std::lerp(static_cast<float>(inZ), static_cast<float>(outZ), frac) * dz;
									rawLoop.emplace_back(crossX, crossZ);

									vx += edx[dir];
									vz += edz[dir];
									if (vx == svx && vz == svz && edgeMask[static_cast<size_t>(vz) * vW + vx] == 0)
										break;

									const uint8_t nextMask = edgeMask[static_cast<size_t>(vz) * vW + vx];
									if (nextMask == 0)
										break;

									const int turnOrder[4] = { (dir + 1) & 3, dir, (dir + 3) & 3, (dir + 2) & 3 };
									int nextDir = -1;
									for (int cand : turnOrder)
									{
										if (nextMask & (1u << cand))
										{
											nextDir = cand;
											break;
										}
									}
									if (nextDir < 0)
										break;
									dir = nextDir;
								}

								if (rawLoop.size() < 8)
									continue;

								// Cyclic smoothing to eliminate any remaining grid staircase along shallow slopes
								const size_t nRaw = rawLoop.size();
								for (int pass = 0; pass < 12; pass++)
								{
									std::vector<Vec2f> tmp(nRaw);
									for (size_t k = 0; k < nRaw; k++)
									{
										const Vec2f& pPrev = rawLoop[(k + nRaw - 1) % nRaw];
										const Vec2f& pCurr = rawLoop[k];
										const Vec2f& pNext = rawLoop[(k + 1) % nRaw];
										tmp[k] = Vec2f(
											0.25f * pPrev.x + 0.5f * pCurr.x + 0.25f * pNext.x,
											0.25f * pPrev.y + 0.5f * pCurr.y + 0.25f * pNext.y);
									}
									rawLoop.swap(tmp);
								}

								// Subsample closed loop to ~0.60 mm spacing and verify minimum perimeter
								std::vector<Vec2f> loopPts;
								loopPts.reserve(nRaw / 2 + 1);
								loopPts.push_back(rawLoop[0]);
								float perimMm = 0.0f;
								for (size_t k = 1; k < nRaw; k++)
								{
									perimMm += (rawLoop[k] - rawLoop[k - 1]).length();
									if ((rawLoop[k] - loopPts.back()).length_sqr() >= 0.60f * 0.60f)
										loopPts.push_back(rawLoop[k]);
								}
								perimMm += (rawLoop[0] - rawLoop.back()).length();
								if (perimMm < 3.0f || loopPts.size() < 6)
									continue;
								if ((loopPts.front() - loopPts.back()).length_sqr() < 0.20f * 0.20f)
									loopPts.pop_back();
								if (loopPts.size() < 6)
									continue;

								// Start the contour at the closest collision-free point from the current tool position
								const Vec3f curPos = waypoints.empty() ? Vec3f(0.0f, entryAirY, 0.0f) : waypoints.back().ToVec3f();
								size_t bestStart = 0;
								bool bestDirect = false;
								float bestDistSq = std::numeric_limits<float>::max();
								for (size_t k = 0; k < loopPts.size(); k++)
								{
									const float candY = getFloorY(loopPts[k].x, loopPts[k].y);
									const Vec3f cand(loopPts[k].x, candY, loopPts[k].y);
									const bool direct = !waypoints.empty() &&
										std::abs(curPos.y - candY) <= 0.45f &&
										isStraightSegmentCollisionFree(curPos, cand, globalItoGrid, 0.05f);
									const float dSq = (cand - curPos).length_sqr();
									if ((direct && !bestDirect) || (direct == bestDirect && dSq < bestDistSq))
									{
										bestDirect = direct;
										bestDistSq = dSq;
										bestStart = k;
									}
								}

								const float startY = getFloorY(loopPts[bestStart].x, loopPts[bestStart].y);
								const float3 startPt(loopPts[bestStart].x, startY, loopPts[bestStart].y);
								if (bestDirect)
									appendSafePoint(startPt);
								else
									transitionSafeCl(startPt);

								const size_t nLoop = loopPts.size();
								for (size_t step = 1; step <= nLoop; step++)
								{
									const Vec2f& lp = loopPts[(bestStart + step) % nLoop];
									appendSafePoint(float3(lp.x, getFloorY(lp.x, lp.y), lp.y));
								}
							}
						}
					}
				}
			}
		}
	}

	// 8. Ensure the first and last waypoints of the toolpath are at safeHeight in air above the block
	if (!waypoints.empty())
	{
		if (waypoints.front().y < safeY - 1e-3f)
		{
			const float3 first = waypoints.front();
			waypoints.insert(waypoints.begin(), float3(first.x + 3.0f, safeY, first.z));
		}

		const float3& last = waypoints.back();
		if (last.y < safeY - 1e-3f)
		{
			waypoints.emplace_back(last.x, safeY, last.z);
		}
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
	double totalLengthMm = 0.0;
	Vec3f prevGCodePt = SceneToGCodeCoords(sceneMmWaypoints.front()).ToVec3f();
	for (const auto& scenePt : sceneMmWaypoints)
	{
		float3 gcodePt = SceneToGCodeCoords(scenePt);
		Vec3f currGCodeVec = gcodePt.ToVec3f();
		totalLengthMm += static_cast<double>((currGCodeVec - prevGCodePt).length());
		prevGCodePt = currGCodeVec;

		file << "N" << lineNum++
			<< "G01"
			<< "X" << gcodePt.x
			<< "Y" << gcodePt.y
			<< "Z" << gcodePt.z
			<< "\n";
	}

	if (file.good())
	{
		const std::filesystem::path gcodePath(filepath);
		const std::filesystem::path logPath = gcodePath.has_parent_path()
			? (gcodePath.parent_path() / "path_lengths.log")
			: std::filesystem::path("path_lengths.log");
		std::ofstream logFile(logPath, std::ios::out | std::ios::app);
		if (logFile.is_open())
		{
			logFile << std::fixed << std::setprecision(3)
				<< gcodePath.filename().string()
				<< " | total_length_mm = " << totalLengthMm
				<< " | waypoints = " << sceneMmWaypoints.size()
				<< "\n";
		}
	}

	return file.good();
}
