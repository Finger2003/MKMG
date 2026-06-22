#include "pch.h"
#include "Intersection.h"
#include "../MathLib/Vec2f.h"


Intersection::Intersection(std::vector<MathLib::Vec4f>&& params, std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2, MathLib::Vec4f&& maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2)
	: SceneObject("Intersection" + std::to_string(s_nextId++), ObjectType::Intersection),
	m_params(std::move(params)), m_surface1(std::move(surface1)), m_surface2(std::move(surface2)), 
	m_maxDomains(std::move(maxDomains)), m_trimModeS1(trimModeS1), m_trimModeS2(trimModeS2)
{
}

void Intersection::InitGeometry(const std::vector<MathLib::Vec3f>& points, const DxDevice& device)
{
	if (points.empty())
		return;

	m_vertexCount = static_cast<UINT>(points.size());
	std::vector<VertexPosition> vertices;
	vertices.reserve(points.size());
	std::transform(points.begin(), points.end(), std::back_inserter(vertices), [](const MathLib::Vec3f& p) {
		return ToVertexPosition(p);
		});

	m_vertexBuffer = device.CreateVertexBuffer(vertices);

	std::vector<VertexPosition> lines1 = GenerateLinesInUVSpace(true);
	if (!lines1.empty())
	{
		m_uvLineCount1 = static_cast<UINT>(lines1.size());
		m_uvLinesBuffer1 = device.CreateVertexBuffer(lines1);
	}

	std::vector<VertexPosition> lines2 = GenerateLinesInUVSpace(false);
	if (!lines2.empty())
	{
		m_uvLineCount2 = static_cast<UINT>(lines2.size());
		m_uvLinesBuffer2 = device.CreateVertexBuffer(lines2);
	}

	m_trimTexture1.Init(device);
	m_trimTexture2.Init(device);

	std::vector<VertexPosition> trimPolys1 = GenerateTrimPolygon(true);
	if (!trimPolys1.empty())
	{
		m_trimPolygonCount1 = static_cast<UINT>(trimPolys1.size());
		m_trimPolygonBuffer1 = device.CreateVertexBuffer(trimPolys1);
	}

	std::vector<VertexPosition> trimPolys2 = GenerateTrimPolygon(false);
	if (!trimPolys2.empty())
	{
		m_trimPolygonCount2 = static_cast<UINT>(trimPolys2.size());
		m_trimPolygonBuffer2 = device.CreateVertexBuffer(trimPolys2);
	}
}

std::vector<VertexPosition> Intersection::GenerateLinesInUVSpace(bool isSurface1)
{
	std::vector<VertexPosition> lines;

	float maxU = isSurface1 ? m_maxDomains.x : m_maxDomains.z;
	float maxV = isSurface1 ? m_maxDomains.y : m_maxDomains.w;

	float thresholdU = maxU * 0.5f;
	float thresholdV = maxV * 0.5f;

	auto toNDC = [](float val, float maxVal) {
		return (val / maxVal) * 2.0f - 1.0f;
	};

	for (size_t i = 0; i < m_params.size() - 1; i++)
	{
		float u1 = isSurface1 ? m_params[i].x : m_params[i].z;
		float v1 = isSurface1 ? m_params[i].y : m_params[i].w;

		float u2 = isSurface1 ? m_params[i + 1].x : m_params[i + 1].z;
		float v2 = isSurface1 ? m_params[i + 1].y : m_params[i + 1].w;

		float offsetU = 0.0f;
		if (u2 - u1 > thresholdU) 
			offsetU = -maxU;
		else if (u1 - u2 > thresholdU) 
			offsetU = maxU;

		float offsetV = 0.0f;
		if (v2 - v1 > thresholdV) 
			offsetV = -maxV;
		else if (v1 - v2 > thresholdV) 
			offsetV = maxV;

		if (offsetU != 0.0f || offsetV != 0.0f)
		{
			// 1. Line leaving the edge (from P1 to a shifted P2)
			lines.push_back({ toNDC(u1, maxU), toNDC(v1, maxV), 0.0f });
			lines.push_back({ toNDC(u2 + offsetU, maxU), toNDC(v2 + offsetV, maxV), 0.0f });

			// 2. Line entering the opposite edge (from a shifted P1 to P2)
			lines.push_back({ toNDC(u1 - offsetU, maxU), toNDC(v1 - offsetV, maxV), 0.0f });
			lines.push_back({ toNDC(u2, maxU), toNDC(v2, maxV), 0.0f });
		}
		else
		{
			lines.push_back({ toNDC(u1, maxU), toNDC(v1, maxV), 0.0f });
			lines.push_back({ toNDC(u2, maxU), toNDC(v2, maxV), 0.0f });
		}
	}

	return lines;
}

std::vector<VertexPosition> Intersection::GenerateTrimPolygon(bool isSurface1)
{
	std::vector<VertexPosition> triangles;
	TrimFillMode mode = isSurface1 ? m_trimModeS1 : m_trimModeS2;
	if (m_params.size() < 2 || mode == TrimFillMode::None)
		return triangles;

	float maxU = isSurface1 ? m_maxDomains.x : m_maxDomains.z;
	float maxV = isSurface1 ? m_maxDomains.y : m_maxDomains.w;

	auto toNDC = [](float val, float maxVal) { return (val / maxVal) * 2.0f - 1.0f; };
	auto getEdge = [maxU, maxV](float u, float v) -> int
		{
			const float edgeEps = std::max(maxU, maxV) * 1e-3f;

			float minD = v;
			int edge = 0; // bottom

			if (maxU - u < minD)
			{
				minD = maxU - u;
				edge = 1; // right
			}

			if (maxV - v < minD)
			{
				minD = maxV - v;
				edge = 2; // top
			}

			if (u < minD)
			{
				minD = u;
				edge = 3; // left
			}

			return minD <= edgeEps ? edge : -1;
		};

	auto snapToEdge = [maxU, maxV](MathLib::Vec2f p, int edge)
		{
			switch (edge)
			{
			case 0: p.y = 0.0f; break;    // bottom
			case 1: p.x = maxU; break;    // right
			case 2: p.y = maxV; break;    // top
			case 3: p.x = 0.0f; break;    // left
			default: break;
			}

			return p;
		};

	std::vector<MathLib::Vec2f> contour;
	contour.reserve(m_params.size());

	for (const auto& p : m_params)
	{
		contour.push_back({
			isSurface1 ? p.x : p.z,
			isSurface1 ? p.y : p.w
			});
	}

	auto closeThroughBoundary = [&](std::vector<MathLib::Vec2f>& c) -> bool
		{
			if (c.size() < 2)
				return false;

			int startEdge = getEdge(c.front().x, c.front().y);
			int endEdge = getEdge(c.back().x, c.back().y);

			if (startEdge == -1 || endEdge == -1)
				return false;

			c.front() = snapToEdge(c.front(), startEdge);
			c.back() = snapToEdge(c.back(), endEdge);

			MathLib::Vec2f corners[4] =
			{
				{ 0.0f, 0.0f },
				{ maxU, 0.0f },
				{ maxU, maxV },
				{ 0.0f, maxV }
			};

			int e = endEdge;

			while (e != startEdge)
			{
				e = (e + 1) % 4;
				c.push_back(corners[e]);
			}

			c.push_back(c.front());
			return true;
		};

	auto convertWrappedToOpenCut = [&](std::vector<MathLib::Vec2f>& pts, bool wrapU) -> bool
		{
			if (pts.size() < 3)
				return false;

			const float maxA = wrapU ? maxU : maxV;
			const float maxB = wrapU ? maxV : maxU;

			const float closeEps = std::max(maxU, maxV) * 1e-4f;

			if ((pts.front() - pts.back()).length_sqr() < closeEps * closeEps)
				pts.pop_back();

			auto getA = [wrapU](const MathLib::Vec2f& p)
				{
					return wrapU ? p.x : p.y;
				};

			auto getB = [wrapU](const MathLib::Vec2f& p)
				{
					return wrapU ? p.y : p.x;
				};

			auto makePoint = [wrapU](float a, float b)
				{
					return wrapU
						? MathLib::Vec2f(a, b)
						: MathLib::Vec2f(b, a);
				};

			const float thresholdA = maxA * 0.5f;

			int seamIndex = -1;

			for (int i = 0; i < static_cast<int>(pts.size()); i++)
			{
				const auto& p0 = pts[i];
				const auto& p1 = pts[(i + 1) % pts.size()];

				if (std::abs(getA(p1) - getA(p0)) > thresholdA)
				{
					seamIndex = i;
					break;
				}
			}

			if (seamIndex < 0)
				return false;

			const auto a = pts[seamIndex];
			const auto b = pts[(seamIndex + 1) % pts.size()];
			const float aA = getA(a);
			const float bA = getA(b);
			const float aB = getB(a);
			const float bB = getB(b);

			std::vector<MathLib::Vec2f> opened;
			
			if (aA > bA)
			{
				// Crossing maxA -> 0.
				float bUnwrappedA = bA + maxA;
				float t = (maxA - aA) / (bUnwrappedA - aA);
				float seamB = std::lerp(aB, bB, t);

				opened.push_back(makePoint(0.0f, seamB));

				for (int k = seamIndex + 1;
					k <= seamIndex + static_cast<int>(pts.size());
					k++)
				{
					opened.push_back(pts[k % pts.size()]);
				}

				opened.push_back(makePoint(maxA, seamB));
			}
			else
			{
				// Crossing 0 -> maxA.
				float bUnwrappedA = bA - maxA;
				float t = (0.0f - aA) / (bUnwrappedA - aA);
				float seamB = std::lerp(aB, bB, t);

				opened.push_back(makePoint(maxA, seamB));

				for (int k = seamIndex + 1;
					k <= seamIndex + static_cast<int>(pts.size());
					k++)
				{
					opened.push_back(pts[k % pts.size()]);
				}

				opened.push_back(makePoint(0.0f, seamB));

				std::reverse(opened.begin(), opened.end());
			}

			pts = std::move(opened);
			return true;
		};

	if (mode == TrimFillMode::ClosedLoop)
	{
		bool wrappedThroughSeam = convertWrappedToOpenCut(contour, true); // U seam

		if (!wrappedThroughSeam)
			wrappedThroughSeam = convertWrappedToOpenCut(contour, false);

		if (wrappedThroughSeam)
		{
			if (!closeThroughBoundary(contour))
				return triangles;
		}
		else
		{
			contour.push_back(contour.front());
		}
	}
	else if (mode == TrimFillMode::BoundaryToBoundary)
	{
		if (!closeThroughBoundary(contour))
			return triangles;
	}

	auto distPointSegmentSqr = [](MathLib::Vec2f p, MathLib::Vec2f a, MathLib::Vec2f b)
		{
			MathLib::Vec2f ab = b - a;
			float len2 = ab.length_sqr();

			if (len2 < 1e-12f)
				return (p - a).length_sqr();

			float t = MathLib::Vec2f::dot(p - a, ab) / len2;
			t = std::clamp(t, 0.0f, 1.0f);

			MathLib::Vec2f q = a + ab * t;
			return (p - q).length_sqr();
		};

	auto pickSafeAnchor = [&]()
		{
			std::vector<MathLib::Vec2f> candidates =
			{
				{ maxU * 0.5f,  maxV * 0.5f  },
				{ maxU * 0.5f,  maxV * 0.25f },
				{ maxU * 0.5f,  maxV * 0.75f },
				{ maxU * 0.25f, maxV * 0.5f  },
				{ maxU * 0.75f, maxV * 0.5f  },
				{ maxU * 0.25f, maxV * 0.25f },
				{ maxU * 0.75f, maxV * 0.75f },
				{ maxU * 0.25f, maxV * 0.75f },
				{ maxU * 0.75f, maxV * 0.25f }
			};

			float eps = std::max(maxU, maxV) * 1e-3f;
			float eps2 = eps * eps;

			for (const auto& candidate : candidates)
			{
				bool ok = true;

				for (size_t i = 0; i + 1 < contour.size(); i++)
				{
					if (distPointSegmentSqr(candidate, contour[i], contour[i + 1]) < eps2)
					{
						ok = false;
						break;
					}
				}

				if (ok)
					return candidate;
			}

			return MathLib::Vec2f(maxU * 0.37f, maxV * 0.61f);
		};

	MathLib::Vec2f anchor = pickSafeAnchor();

	for (size_t i = 0; i < contour.size() - 1; i++)
	{
		triangles.push_back({
			toNDC(anchor.x, maxU),
			toNDC(anchor.y, maxV),
			0.0f
			});

		triangles.push_back({
			toNDC(contour[i].x, maxU),
			toNDC(contour[i].y, maxV),
			0.0f
			});

		triangles.push_back({
			toNDC(contour[i + 1].x, maxU),
			toNDC(contour[i + 1].y, maxV),
			0.0f
			});
	}

	return triangles;
}
