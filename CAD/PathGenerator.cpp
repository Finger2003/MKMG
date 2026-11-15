#include "pch.h"
#include "PathGenerator.h"
#include "GeometryMath.h"
#include "Surface.h"
#include "Torus.h"
#include <omp.h>
#include <atomic>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <utility>
#include <iomanip>

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
