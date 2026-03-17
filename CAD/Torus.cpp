#include "pch.h"
#include "Torus.h"
#include "DxDevice.h"

using namespace MathLib;
void Torus::SetMajorRadius(float radius)
{
	float r = std::clamp(radius, minorRadius, cMaxMajorRadius);
	if (r != majorRadius)
	{
		majorRadius = r;
		dirty = true;
	}
}

void Torus::SetMinorRadius(float radius)
{
	float r = std::clamp(radius, cMinMinorRadius, majorRadius);
	if (r != minorRadius)
	{
		minorRadius = r;
		dirty = true;
	}
}

void Torus::SetSegments(int major, int minor)
{
	int mSeg = std::clamp(major, cMinMajorSegments, cMaxMajorSegments);
	int nSeg = std::clamp(minor, cMinMinorSegments, cMaxMinorSegments);
	if (mSeg != majorSegments || nSeg != minorSegments)
	{
		majorSegments = mSeg;
		minorSegments = nSeg;
		dirty = true;
	}
}

void Torus::SetScale(float scale)
{
	float s = std::clamp(scale, cMinScale, cMaxScale);
	if (s != m_scale)
	{
		m_scale = s;
		UpdateModelMatrix();
	}
}

void Torus::UpdateMesh(const DxDevice& device)
{
	if (dirty)
	{
		GenerateMesh();
		UpdateBuffers(device);
		dirty = false;
	}
}

void Torus::UpdateBuffers(const DxDevice& device)
{
	m_vertexBuffer = device.CreateVertexBuffer(vertices);
	m_indexBuffer = device.CreateIndexBuffer(indices);
}

void Torus::UpdateModelMatrix()
{
	Mat4f translation = Mat4f::Translation(m_position.x, m_position.y, m_position.z);
	Mat4f scaling = Mat4f::Scaling(m_scale);
	m_modelMatrix = translation * m_rotationMatrix * scaling;
}

void Torus::GenerateMesh()
{
	GenerateVertices();
	GenerateIndices();
}

void Torus::GenerateVertices()
{
	vertices.clear();
	vertices.reserve(static_cast<size_t>(majorSegments) * minorSegments);
	for (int i = 0; i < majorSegments; i++)
	{
		float beta = i * 2.0f * std::numbers::pi_v<float> / majorSegments;
		float cosBeta = std::cos(beta);
		float sinBeta = std::sin(beta);
		for (int j = 0; j < minorSegments; j++)
		{
			float alpha = j * 2.0f * std::numbers::pi_v<float> / minorSegments;
			float cosAlpha = std::cos(alpha);
			float sinAlpha = std::sin(alpha);
			vertices.emplace_back(
				(majorRadius + minorRadius * cosAlpha) * cosBeta,
				minorRadius * sinAlpha,
				-(majorRadius + minorRadius * cosAlpha) * sinBeta
			);
		}
	}
}

void Torus::GenerateIndices()
{
	indices.clear();
	vertices.reserve(static_cast<size_t>(majorSegments) * minorSegments * 2 * 2); // 2 edges per vertex, 2 vertices per edge
	for (int i = 0; i < majorSegments; i++)
	{
		for (int j = 0; j < minorSegments; j++)
		{
			int current = i * minorSegments + j;

			// Minor Circle Edges
			// (i, j) -> (i, j + 1)
			int nextMinor = i * minorSegments + (j + 1) % minorSegments;
			indices.push_back(current);
			indices.push_back(nextMinor);

			// Major Circle Edges
			// (i, j) -> (i + 1, j)
			int nextMajor = ((i + 1) % majorSegments) * minorSegments + j;
			indices.push_back(current);
			indices.push_back(nextMajor);
		}
	}
}