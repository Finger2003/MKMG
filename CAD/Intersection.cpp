#include "pch.h"
#include "Intersection.h"


Intersection::Intersection(std::vector<MathLib::Vec4f>&& params, std::weak_ptr<SceneObject> surface1, std::weak_ptr<SceneObject> surface2, MathLib::Vec4f&& maxDomains)
	: SceneObject("Intersection" + std::to_string(s_nextId++), ObjectType::Intersection),
	m_params(std::move(params)), m_surface1(std::move(surface1)), m_surface2(std::move(surface2)), 
	m_maxDomains(std::move(maxDomains))
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

	std::vector<VertexPosition> lines1 = GenerateLinesInUVSpace(device, true);
	if (!lines1.empty())
	{
		m_uvLineCount1 = static_cast<UINT>(lines1.size());
		m_uvLinesBuffer1 = device.CreateVertexBuffer(lines1);
	}

	std::vector<VertexPosition> lines2 = GenerateLinesInUVSpace(device, false);
	if (!lines2.empty())
	{
		m_uvLineCount2 = static_cast<UINT>(lines2.size());
		m_uvLinesBuffer2 = device.CreateVertexBuffer(lines2);
	}

	m_trimTexture1.Init(device);
	m_trimTexture2.Init(device);
}

std::vector<VertexPosition> Intersection::GenerateLinesInUVSpace(const DxDevice& device, bool isSurface1)
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
