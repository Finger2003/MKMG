#include "pch.h"
#include "InterpolatingCurve.h"
#include "../MathLib/Vec3f.h"

using namespace std;
using namespace MathLib;

//unsigned int InterpolatingCurve::s_nextId = 0;

InterpolatingCurve::InterpolatingCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
	: Curve("InterpolatingCurve" + std::to_string(s_nextId++), ObjectType::InterpolatingCurve, std::move(controlPoints))
{}

InterpolatingCurve::InterpolatingCurve(unsigned int id, std::vector<std::weak_ptr<Point>> && controlPoints, std::optional<ParsedNameData> && nameData)
	: Base(id, nameData ? std::move(nameData->name) : "InterpolatingCurve" + std::to_string(s_nextId++), ObjectType::InterpolatingCurve, std::move(controlPoints))
{
	if (nameData)
		AdvanceCounter(nameData->index);
}

//InterpolatingCurve::InterpolatingCurve(unsigned int id, unsigned int interpolatingCurveIndex, std::string && name, std::vector<std::weak_ptr<Point>> && controlPoints)
//	: Base(id, std::move(name), ObjectType::InterpolatingCurve, std::move(controlPoints))
//{
//	AdvanceCounter(interpolatingCurveIndex);
//}

void InterpolatingCurve::UpdatePolyline(const DxDevice & device)
{
	CleanExpiredPoints();
	if (!m_isDirty)
		return;

	if (m_controlPoints.size() < 2)
	{
		m_lineVertexCount = 0;
		m_curveVertexCount = 0;
		m_isDirty = false;
		return;
	}

	//bool needUpdate = false;
	//if (m_lastPositions.size() != m_controlPoints.size())
	//	needUpdate = true;
	//else
	//{
	//	for (size_t i = 0; i < m_controlPoints.size(); i++)
	//	{
	//		auto sp = m_controlPoints[i].lock();
	//		if (!sp ||
	//			sp->m_position.x != m_lastPositions[i].x ||
	//			sp->m_position.y != m_lastPositions[i].y ||
	//			sp->m_position.z != m_lastPositions[i].z)
	//		{
	//			needUpdate = true;
	//			break;
	//		}
	//	}
	//}
	std::vector<Vec3f> currentPositions;
	currentPositions.reserve(m_controlPoints.size());
	for (const auto& wp : m_controlPoints)
	{
		if (auto sp = wp.lock())
			currentPositions.push_back(sp->m_position.ToVec3f());
	}

	//if (needUpdate)
	//{
		//m_lastPositions.clear();
		//for (const auto& wp : m_controlPoints)
		//{
		//	if (auto sp = wp.lock())
		//		m_lastPositions.push_back(sp->m_position);
		//}

		//size_t raw_n = m_lastPositions.size();
		size_t raw_n = currentPositions.size();
		std::vector<Vec3f> P;
		P.reserve(raw_n);

		for (size_t i = 0; i < raw_n; i++)
		{
			//Vec3f pt = m_lastPositions[i].ToVec3f();
			Vec3f pt = currentPositions[i];
			if (P.empty() || (pt - P.back()).length_sqr() > 1e-8f)
				P.push_back(pt);
		}
		size_t n = P.size();

		if (n < 2)
		{
			m_lineVertexCount = 0;
			m_curveVertexCount = 0;
			m_isDirty = false;
			return;
		}

		// Control polygon
		std::vector<VertexPosition> lineVertices;
		lineVertices.reserve(n);
		//for (size_t i = 0; i < n; i++)
		//	lineVertices.push_back({ m_lastPositions[i].x, m_lastPositions[i].y, m_lastPositions[i].z });
		for (size_t i = 0; i < n; i++)
			lineVertices.push_back(ToVertexPosition(P[i])); 

		// Calculate chordal lengths
		std::vector<float> h(n - 1);
		for (size_t i = 0; i < n - 1; i++)
			h[i] = (P[i + 1] - P[i]).length(); // Avoid division by zero

		std::vector<float> a(n), b(n), c(n);
		std::vector<Vec3f> d(n);

		// Natural boundary conditions start
		b[0] = 2.0f;
		c[0] = 1.0f;
		d[0] = (P[1] - P[0]) * (3.0f / h[0]);

		// C2 continuity interior nodes
		for (size_t i = 1; i < n - 1; i++)
		{
			a[i] = h[i];
			b[i] = 2.0f * (h[i - 1] + h[i]);
			c[i] = h[i - 1];
			d[i] = (P[i] - P[i - 1]) * (3.0f * h[i] / h[i - 1]) + (P[i + 1] - P[i]) * (3.0f * h[i - 1] / h[i]);
		}

		// Natural boundary conditions end
		a[n - 1] = 1.0f;
		b[n - 1] = 2.0f;
		d[n - 1] = (P[n - 1] - P[n - 2]) * (3.0f / h[n - 2]);


		// Forward Sweep
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

		// Back Substitution
		std::vector<Vec3f> D(n); // Tangent vectors at every point
		D[n - 1] = d_prime[n - 1];
		for (int i = static_cast<int>(n - 2); i >= 0; i--)
			D[i] = d_prime[i] - D[i + 1] * c_prime[i];

		// Convert tangents to Bezier control points
		std::vector<VertexPosition> vertices;
		vertices.reserve((n - 1) * 4);
		for (size_t i = 0; i < n - 1; i++)
		{
			Vec3f b0 = P[i];
			Vec3f b1 = P[i] + D[i] * (h[i] / 3.0f);
			Vec3f b2 = P[i + 1] - D[i + 1] * (h[i] / 3.0f);
			Vec3f b3 = P[i + 1];

			vertices.push_back({ b0.x, b0.y, b0.z });
			vertices.push_back({ b1.x, b1.y, b1.z });
			vertices.push_back({ b2.x, b2.y, b2.z });
			vertices.push_back({ b3.x, b3.y, b3.z });
		}

		UpdateBuffer(device, m_lineVertexBuffer, m_lineBufferCapacity, lineVertices, m_lineVertexCount, 2);
		UpdateBuffer(device, m_curveVertexBuffer, m_curveBufferCapacity, vertices, m_curveVertexCount, 4);
	//}	
		m_isDirty = false;
}
