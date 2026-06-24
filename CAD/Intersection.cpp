#include "pch.h"
#include "Intersection.h"
#include "../MathLib/Vec2f.h"

using namespace MathLib;


Intersection::Intersection(std::string&& name, std::vector<MathLib::Vec3f>&& basePoints, std::vector<MathLib::Vec4f>&& params, std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2, MathLib::Vec4f&& maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2, bool isClosedLoop, ObjectType type)
	: SceneObject(std::move(name), type),
	m_basePoints(std::move(basePoints)), m_params(std::move(params)), m_surface1(std::move(surface1)), m_surface2(std::move(surface2)),
	m_maxDomains(std::move(maxDomains)), m_trimModeS1(trimModeS1), m_trimModeS2(trimModeS2), m_isClosedLoop(isClosedLoop)
{}

void Intersection::InitGeometry(const DxDevice& device)
{
	if (m_basePoints.empty())
		return;


	//m_vertexCount = static_cast<UINT>(m_basePoints.size());
	//std::vector<VertexPosition> vertices;
	//vertices.reserve(points.size());
	//std::transform(points.begin(), points.end(), std::back_inserter(vertices), [](const MathLib::Vec3f& p) {
	//	return ToVertexPosition(p);
	//	});

	//m_vertexBuffer = device.CreateVertexBuffer(vertices);

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

	auto toNDCU = [](float val, float maxVal) { return (val / maxVal) * 2.0f - 1.0f; };
	auto toNDCV = [](float val, float maxVal) { return 1.0f - (val / maxVal) * 2.0f; };

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
			lines.push_back({ toNDCU(u1, maxU), toNDCV(v1, maxV), 0.0f });
			lines.push_back({ toNDCU(u2 + offsetU, maxU), toNDCV(v2 + offsetV, maxV), 0.0f });

			// 2. Line entering the opposite edge (from a shifted P1 to P2)
			lines.push_back({ toNDCU(u1 - offsetU, maxU), toNDCV(v1 - offsetV, maxV), 0.0f });
			lines.push_back({ toNDCU(u2, maxU), toNDCV(v2, maxV), 0.0f });
		}
		else
		{
			lines.push_back({ toNDCU(u1, maxU), toNDCV(v1, maxV), 0.0f });
			lines.push_back({ toNDCU(u2, maxU), toNDCV(v2, maxV), 0.0f });
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

	//auto toNDC = [](float val, float maxVal) { return (val / maxVal) * 2.0f - 1.0f; };
	auto toNDCU = [](float val, float maxVal) { return (val / maxVal) * 2.0f - 1.0f; };
	auto toNDCV = [](float val, float maxVal) { return 1.0f - (val / maxVal) * 2.0f; };
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

	auto snapToEdge = [maxU, maxV](Vec2f p, int edge)
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

	std::vector<Vec2f> contour;
	contour.reserve(m_params.size());

	for (const auto& p : m_params)
	{
		contour.push_back({
			isSurface1 ? p.x : p.z,
			isSurface1 ? p.y : p.w
			});
	}

	auto closeThroughBoundary = [&](std::vector<Vec2f>& c) -> bool
		{
			if (c.size() < 2)
				return false;

			int startEdge = getEdge(c.front().x, c.front().y);
			int endEdge = getEdge(c.back().x, c.back().y);

			if (startEdge == -1 || endEdge == -1)
				return false;

			c.front() = snapToEdge(c.front(), startEdge);
			c.back() = snapToEdge(c.back(), endEdge);

			Vec2f corners[4] =
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

	auto convertWrappedToOpenCut = [&](std::vector<Vec2f>& pts, bool wrapU) -> bool
		{
			if (pts.size() < 3)
				return false;

			const float maxA = wrapU ? maxU : maxV;
			const float maxB = wrapU ? maxV : maxU;

			const float closeEps = std::max(maxU, maxV) * 1e-4f;

			if ((pts.front() - pts.back()).length_sqr() < closeEps * closeEps)
				pts.pop_back();

			auto getA = [wrapU](const Vec2f& p)
				{
					return wrapU ? p.x : p.y;
				};

			auto getB = [wrapU](const Vec2f& p)
				{
					return wrapU ? p.y : p.x;
				};

			auto makePoint = [wrapU](float a, float b)
				{
					return wrapU
						? Vec2f(a, b)
						: Vec2f(b, a);
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

			std::vector<Vec2f> opened;

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
		int crossU = 0, crossV = 0;
		const float thresholdU = maxU * 0.5f;
		const float thresholdV = maxV * 0.5f;
		for (size_t i = 0; i < contour.size(); i++)
		{
			auto p0 = contour[i];
			auto p1 = contour[(i + 1) % contour.size()];
			if (std::abs(p1.x - p0.x) > thresholdU) crossU++;
			if (std::abs(p1.y - p0.y) > thresholdV) crossV++;
		}

		bool isEquatorU = (crossU % 2 != 0);
		bool isEquatorV = (crossV % 2 != 0);

		if (isEquatorU)
		{
			convertWrappedToOpenCut(contour, true);
			if (!closeThroughBoundary(contour)) return triangles;
		}
		else if (isEquatorV)
		{
			convertWrappedToOpenCut(contour, false);
			if (!closeThroughBoundary(contour)) return triangles;
		}
		else
		{
			std::vector<Vec2f> unwrapped;
			unwrapped.push_back(contour[0]);

			for (size_t i = 1; i < contour.size(); i++)
			{
				Vec2f p = contour[i];
				Vec2f prev = unwrapped.back();

				if (p.x - prev.x > thresholdU) p.x -= maxU;
				else if (prev.x - p.x > thresholdU) p.x += maxU;

				if (p.y - prev.y > thresholdV) p.y -= maxV;
				else if (prev.y - p.y > thresholdV) p.y += maxV;

				unwrapped.push_back(p);
			}
			contour = unwrapped;
			contour.push_back(contour.front()); // Close the loop
		}
	}
	else if (mode == TrimFillMode::BoundaryToBoundary)
	{
		if (!closeThroughBoundary(contour))
			return triangles;
	}

	float minUnwrappedU = contour[0].x;
	float maxUnwrappedU = contour[0].x;
	float minUnwrappedV = contour[0].y;
	float maxUnwrappedV = contour[0].y;
	for (const auto& p : contour)
	{
		minUnwrappedU = std::min(minUnwrappedU, p.x);
		maxUnwrappedU = std::max(maxUnwrappedU, p.x);
		minUnwrappedV = std::min(minUnwrappedV, p.y);
		maxUnwrappedV = std::max(maxUnwrappedV, p.y);
	}

	Vec2f anchor = contour[0];
	std::vector<VertexPosition> baseTriangles;

	for (size_t i = 0; i < contour.size() - 1; i++)
	{
		baseTriangles.push_back({ toNDCU(anchor.x, maxU), toNDCV(anchor.y, maxV), 0.0f });
		baseTriangles.push_back({ toNDCU(contour[i].x, maxU), toNDCV(contour[i].y, maxV), 0.0f });
		baseTriangles.push_back({ toNDCU(contour[i + 1].x, maxU), toNDCV(contour[i + 1].y, maxV), 0.0f });
	}

	int startX = (maxUnwrappedU > maxU) ? -1 : 0;
	int endX = (minUnwrappedU < 0.0f) ? 1 : 0;
	int startY = (maxUnwrappedV > maxV) ? -1 : 0;
	int endY = (minUnwrappedV < 0.0f) ? 1 : 0;

	// Duplicate triangles in a 3x3 grid to ensure seamless texture wrapping
	for (int offsetX = startX; offsetX <= endX; offsetX++)
	{
		for (int offsetY = startY; offsetY <= endY; offsetY++)
		{
			for (const auto& tri : baseTriangles)
			{
				triangles.push_back({
					tri.x + offsetX * 2.0f,
					tri.y - offsetY * 2.0f,
					0.0f
					});
			}
		}
	}

	return triangles;
}

LinearIntersection::LinearIntersection(std::vector<MathLib::Vec3f>&& basePoints, std::vector<MathLib::Vec4f>&& params, std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2, MathLib::Vec4f maxDomains, TrimFillMode trimModeS1, TrimFillMode trimModeS2, bool isClosedLoop)
	: Intersection("LinearIntersection" + std::to_string(s_nextId++), std::move(basePoints), std::move(params), std::move(surface1), std::move(surface2), std::move(maxDomains), trimModeS1, trimModeS2, isClosedLoop, ObjectType::LinearIntersection)
{}

void LinearIntersection::InitGeometry(const DxDevice& device)
{
	Intersection::InitGeometry(device);

	if (m_basePoints.empty())
		return;

	m_vertexCount = static_cast<UINT>(m_basePoints.size());
	std::vector<VertexPosition> vertices;
	vertices.reserve(m_basePoints.size());
	std::transform(m_basePoints.begin(), m_basePoints.end(), std::back_inserter(vertices), [](const MathLib::Vec3f& p) {
		return ToVertexPosition(p);
		});

	m_vertexBuffer = device.CreateVertexBuffer(vertices);
}


BezierIntersection::BezierIntersection(LinearIntersection&& linear)
	: Intersection(std::move("BezierIntersection" + std::to_string(s_nextId++)), std::move(linear.m_basePoints), std::move(linear.m_params), linear.m_surface1, linear.m_surface2, std::move(linear.m_maxDomains), linear.m_trimModeS1, linear.m_trimModeS2, linear.m_isClosedLoop, ObjectType::BezierIntersection)
{
	m_uvLinesBuffer1 = linear.m_uvLinesBuffer1;
	m_uvLinesBuffer2 = linear.m_uvLinesBuffer2;
	m_uvLineCount1 = linear.m_uvLineCount1;
	m_uvLineCount2 = linear.m_uvLineCount2;

	m_trimPolygonBuffer1 = linear.m_trimPolygonBuffer1;
	m_trimPolygonBuffer2 = linear.m_trimPolygonBuffer2;
	m_trimPolygonCount1 = linear.m_trimPolygonCount1;
	m_trimPolygonCount2 = linear.m_trimPolygonCount2;

	m_trimTexture1 = linear.m_trimTexture1;
	m_trimTexture2 = linear.m_trimTexture2;

	m_reverseTrimS1 = linear.m_reverseTrimS1;
	m_reverseTrimS2 = linear.m_reverseTrimS2;
}

void BezierIntersection::InitGeometry(const DxDevice& device)
{
	Intersection::InitGeometry(device);
	InitBezierGeometry(device);
}

void BezierIntersection::InitBezierGeometry(const DxDevice& device)
{
	if (m_basePoints.size() < 2)
		return;

	size_t n = m_basePoints.size();
	std::vector<Vec3f> P = m_basePoints;
	std::vector<Vec3f> D(n);

	//if (m_isClosedLoop && n > 2)
	//{
	//	int unique_n = static_cast<int>(n) - 1;
	//	int pad = std::min(10, unique_n - 1);

	//	std::vector<Vec3f> P_pad;
	//	P_pad.reserve(n + 2 * pad);

	//	// Prefix padding
	//	for (int i = unique_n - pad; i < unique_n; i++)
	//		P_pad.push_back(P[i]);
	//	// Core body
	//	for (int i = 0; i < static_cast<int>(n); i++)
	//		P_pad.push_back(P[i]);
	//	// Suffix padding
	//	for (int i = 1; i <= pad; i++)
	//		P_pad.push_back(P[i]);

	//	size_t n_pad = P_pad.size();
	//	std::vector<float> h(n_pad - 1);
	//	for (size_t i = 0; i < n_pad - 1; i++)
	//		h[i] = (P_pad[i + 1] - P_pad[i]).length();

	//	std::vector<float> a(n_pad), b(n_pad), c(n_pad);
	//	std::vector<Vec3f> d(n_pad);

	//	b[0] = 1.0f; 
	//	c[0] = 1.0f; 
	//	d[0] = (P_pad[1] - P_pad[0]) * (2.0f / h[0]);
	//	for (size_t i = 1; i < n_pad - 1; i++)
	//	{
	//		a[i] = h[i]; 
	//		b[i] = 2.0f * (h[i - 1] + h[i]); 
	//		c[i] = h[i - 1];
	//		d[i] = (P_pad[i] - P_pad[i - 1]) * (3.0f * h[i] / h[i - 1]) + (P_pad[i + 1] - P_pad[i]) * (3.0f * h[i - 1] / h[i]);
	//	}
	//	a[n_pad - 1] = 1.0f;
	//	b[n_pad - 1] = 1.0f;
	//	d[n_pad - 1] = (P_pad[n_pad - 1] - P_pad[n_pad - 2]) * (2.0f / h[n_pad - 2]);

	//	std::vector<float> c_prime(n_pad);
	//	std::vector<MathLib::Vec3f> d_prime(n_pad);
	//	c_prime[0] = c[0] / b[0]; d_prime[0] = d[0] * (1.0f / b[0]);

	//	for (size_t i = 1; i < n_pad; i++)
	//	{
	//		float m = 1.0f / (b[i] - a[i] * c_prime[i - 1]);
	//		c_prime[i] = c[i] * m;
	//		d_prime[i] = (d[i] - d_prime[i - 1] * a[i]) * m;
	//	}

	//	std::vector<MathLib::Vec3f> D_pad(n_pad);
	//	D_pad[n_pad - 1] = d_prime[n_pad - 1];
	//	for (int i = static_cast<int>(n_pad - 2); i >= 0; i--)
	//		D_pad[i] = d_prime[i] - D_pad[i + 1] * c_prime[i];

	//	for (int i = 0; i < static_cast<int>(n); i++)
	//		D[i] = D_pad[i + pad];
	//	D[n - 1] = D[0];
	//}
	if (m_isClosedLoop && n > 2)
	{
		int unique_n = static_cast<int>(n) - 1;

		std::vector<float> h(unique_n);
		for (int i = 0; i < unique_n; i++)
			h[i] = (P[(i + 1) % unique_n] - P[i]).length();

		std::vector<float> a(unique_n), b(unique_n), c(unique_n);
		std::vector<Vec3f> d(unique_n);

		for (int i = 0; i < unique_n; i++)
		{
			int prev = (i + unique_n - 1) % unique_n;
			a[i] = h[i];
			b[i] = 2.0f * (h[prev] + h[i]);
			c[i] = h[prev];
			d[i] = (P[i] - P[prev]) * (3.0f * h[i] / h[prev]) + (P[(i + 1) % unique_n] - P[i]) * (3.0f * h[prev] / h[i]);
		}

		float gamma = -b[0];

		std::vector<float> a_prime = a, b_prime = b, c_prime = c;
		b_prime[0] -= gamma;
		b_prime[unique_n - 1] -= a[0] * c[unique_n - 1] / gamma;

		std::vector<float> u(unique_n, 0.0f);
		u[0] = gamma;
		u[unique_n - 1] = c[unique_n - 1];

		std::vector<float> v(unique_n, 0.0f);
		v[0] = 1.0f;
		v[unique_n - 1] = a[0] / gamma;

		auto SolveThomas = [&](const auto& d_input, auto& out) {
			std::vector<float> c_star(unique_n);
			out.resize(unique_n);

			c_star[0] = c_prime[0] / b_prime[0];
			out[0] = d_input[0] * (1.0f / b_prime[0]);

			for (int i = 1; i < unique_n; i++)
			{
				float m = 1.0f / (b_prime[i] - a_prime[i] * c_star[i - 1]);
				c_star[i] = c_prime[i] * m;
				out[i] = (d_input[i] - out[i - 1] * a_prime[i]) * m;
			}
			for (int i = unique_n - 2; i >= 0; i--)
			{
				out[i] = out[i] - out[i + 1] * c_star[i];
			}
			};

		std::vector<Vec3f> Y;
		SolveThomas(d, Y);

		std::vector<float> Q;
		SolveThomas(u, Q);

		Vec3f v_dot_y = Y[0] + Y[unique_n - 1] * v[unique_n - 1];
		float v_dot_q = Q[0] + Q[unique_n - 1] * v[unique_n - 1];
		float scalar_ratio = 1.0f / (1.0f + v_dot_q);

		for (int i = 0; i < unique_n; i++)
			D[i] = Y[i] - v_dot_y * (Q[i] * scalar_ratio);

		D[n - 1] = D[0];
	}
	else
	{
		std::vector<float> h(n - 1);
		for (size_t i = 0; i < n - 1; i++)
			h[i] = (P[i + 1] - P[i]).length();

		std::vector<float> a(n), b(n), c(n);
		std::vector<Vec3f> d(n);

		b[0] = 1.0f;
		c[0] = 1.0f;
		d[0] = (P[1] - P[0]) * (2.0f / h[0]);

		for (size_t i = 1; i < n - 1; i++)
		{
			a[i] = h[i]; 
			b[i] = 2.0f * (h[i - 1] + h[i]); 
			c[i] = h[i - 1];
			d[i] = (P[i] - P[i - 1]) * (3.0f * h[i] / h[i - 1]) + (P[i + 1] - P[i]) * (3.0f * h[i - 1] / h[i]);
		}

		a[n - 1] = 1.0f;
		b[n - 1] = 1.0f;
		d[n - 1] = (P[n - 1] - P[n - 2]) * (2.0f / h[n - 2]);

		std::vector<float> c_prime(n);
		std::vector<Vec3f> d_prime(n);
		c_prime[0] = c[0] / b[0];
		d_prime[0] = d[0] * (1.0f / b[0]);

		for (size_t i = 1; i < n; i++)
		{
			float m = 1.0f / (b[i] - a[i] * c_prime[i - 1]);
			c_prime[i] = c[i] * m;
			d_prime[i] = (d[i] - d_prime[i - 1] * a[i]) * m;
		}

		D[n - 1] = d_prime[n - 1];
		for (int i = static_cast<int>(n - 2); i >= 0; i--)
			D[i] = d_prime[i] - D[i + 1] * c_prime[i];
	}

	std::vector<VertexPosition> bezierVertices;
	bezierVertices.reserve((n - 1) * 4);
	for (size_t i = 0; i < n - 1; i++)
	{
		float hi = (P[i + 1] - P[i]).length();
		Vec3f b0 = P[i];
		Vec3f b1 = P[i] + D[i] * (hi / 3.0f);
		Vec3f b2 = P[i + 1] - D[i + 1] * (hi / 3.0f);
		Vec3f b3 = P[i + 1];

		bezierVertices.push_back({ b0.x, b0.y, b0.z });
		bezierVertices.push_back({ b1.x, b1.y, b1.z });
		bezierVertices.push_back({ b2.x, b2.y, b2.z });
		bezierVertices.push_back({ b3.x, b3.y, b3.z });
	}

	m_bezierVertexCount = static_cast<UINT>(bezierVertices.size());
	m_bezierVertexBuffer = device.CreateVertexBuffer(bezierVertices);
}
