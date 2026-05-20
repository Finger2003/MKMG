#include "pch.h"
#include "Torus.h"
#include "DxDevice.h"

using namespace std;
using namespace MathLib;

Torus::Torus(float3 position) : TransformableObject(position, "Torus" + to_string(s_nextId++), ObjectType::Torus)//, m_position(position)
{
	UpdateModelMatrix();
}

Torus::Torus(unsigned int id, float3 position, float3 scale, const MathLib::Mat4f& rotationMatrix, float majorRadius, float minorRadius, uint2 samples, std::optional<ParsedNameData>&& nameData)
	: Base(id, position, nameData ? std::move(nameData->name) : "Torus" + to_string(s_nextId++), ObjectType::Torus),
	m_scale(scale), m_baseScale(scale), m_rotationMatrix(rotationMatrix), m_baseRotationMatrix(rotationMatrix),
	majorRadius(majorRadius), minorRadius(minorRadius), majorSegments(samples.u), minorSegments(samples.v)
{
	if (nameData)
		AdvanceCounter(nameData->index);
	m_eulerAngles = Mat4f::ExtractEulerAngles(m_rotationMatrix);
	UpdateModelMatrix();
}


void Torus::SetMajorRadius(float radius)
{
	float r = clamp(radius, minorRadius, cMaxMajorRadius);
	if (r != majorRadius)
	{
		majorRadius = r;
		dirty = true;
	}
}

void Torus::SetMinorRadius(float radius)
{
	float r = clamp(radius, cMinMinorRadius, majorRadius);
	if (r != minorRadius)
	{
		minorRadius = r;
		dirty = true;
	}
}

void Torus::SetSegments(int major, int minor)
{
	int mSeg = clamp(major, cMinMajorSegments, cMaxMajorSegments);
	int nSeg = clamp(minor, cMinMinorSegments, cMaxMinorSegments);
	if (mSeg != majorSegments || nSeg != minorSegments)
	{
		majorSegments = mSeg;
		minorSegments = nSeg;
		dirty = true;
	}
}

void Torus::SetScale(const float3& scale)
{
	if (scale.x != m_scale.x || scale.y != m_scale.y || scale.z != m_scale.z)
	{
		m_scale = scale;
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
	//Mat4f scaling = Mat4f::Scaling(m_scale);
	Mat4f scaling = Mat4f::Scaling(m_scale.x, m_scale.y, m_scale.z);
	m_modelMatrix = translation * m_rotationMatrix * scaling;
}

nlohmann::json Torus::Serialize() const
{
	nlohmann::json j = Base::Serialize();
	j["rotation"] = m_rotationMatrix.ToQuaternion();
	j["scale"] = m_scale;
	j["samples"] = uint2{ static_cast<uint32_t>(majorSegments), static_cast<uint32_t>(minorSegments) };
	j["smallRadius"] = minorRadius;
	j["largeRadius"] = majorRadius;
	return j;
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
		float beta = i * 2.0f * numbers::pi_v<float> / majorSegments;
		float cosBeta = cos(beta);
		float sinBeta = sin(beta);
		for (int j = 0; j < minorSegments; j++)
		{
			float alpha = j * 2.0f * numbers::pi_v<float> / minorSegments;
			float cosAlpha = cos(alpha);
			float sinAlpha = sin(alpha);
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