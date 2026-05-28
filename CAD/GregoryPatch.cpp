#include "pch.h"
#include "GregoryPatch.h"
#include "Point.h"
#include "CadApplication.h"
#include "HoleDetection.h"
#include "DxDevice.h"
#include "../MathLib/Vec3f.h"

using namespace MathLib;

GregoryPatch::GregoryPatch(Hole3Cycle&& hole)
	: Base("GregoryPatch" + std::to_string(s_nextId++), ObjectType::GregoryPatch), m_hole(std::move(hole))
{
	//m_controlPoints.reserve(24);
	//for (int i = 0; i < 3; i++)
	//{
	//	const auto& edge = hole.edges[i];
	//	m_controlPoints.insert(m_controlPoints.end(), edge.boundaryPoints.begin(), edge.boundaryPoints.end());
	//	m_controlPoints.insert(m_controlPoints.end(), edge.innerPoints.begin(), edge.innerPoints.end());
	//}
}

void GregoryPatch::InitGeometry(const DxDevice& device)
{

	m_vertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(m_vertexCount);
	m_continuityVertexBuffer = device.CreateDynamicVertexBuffer<VertexPosition>(m_continuityVertexCount);
}

void GregoryPatch::UpdateVertices(const DxDevice& device)
{
	if (!m_isDirty)
		return;

	auto getPos = [](const std::weak_ptr<Point>& pt) -> Vec3f {
		auto sharedPt = pt.lock();
		return sharedPt ? sharedPt->m_position.ToVec3f() : Vec3f(0, 0, 0);
		};

	Point* corners[3];
	corners[0] = (m_hole.edges[2].startPoint == m_hole.edges[0].startPoint || m_hole.edges[2].startPoint == m_hole.edges[0].endPoint) ? m_hole.edges[2].startPoint : m_hole.edges[2].endPoint;
	corners[1] = (m_hole.edges[0].startPoint == m_hole.edges[1].startPoint || m_hole.edges[0].startPoint == m_hole.edges[1].endPoint) ? m_hole.edges[0].startPoint : m_hole.edges[0].endPoint;
	corners[2] = (m_hole.edges[1].startPoint == m_hole.edges[2].startPoint || m_hole.edges[1].startPoint == m_hole.edges[2].endPoint) ? m_hole.edges[1].startPoint : m_hole.edges[1].endPoint;

	Vec3f B_left[3][4], B_right[3][4];
	Vec3f I_left[3][4], I_right[3][4];
	Vec3f P3[3], P2[3], Q[3], P1[3];
	Vec3f P_center(0, 0, 0);

	for (int i = 0; i < 3; i++)
	{
		const auto& edge = m_hole.edges[i];
		bool isForward = (edge.boundaryPoints[0].lock().get() == corners[i]);

		Vec3f B[4], I[4];
		for (int j = 0; j < 4; j++)
		{
			int idx = isForward ? j : (3 - j);
			B[j] = getPos(edge.boundaryPoints[idx]);
			I[j] = getPos(edge.innerPoints[idx]);
		}

		// Subdivide Boundary B at t=0.5
		B_left[i][0] = B[0];
		B_left[i][1] = (B[0] + B[1]) * 0.5f;
		Vec3f B_L2 = (B[1] + B[2]) * 0.5f;
		B_left[i][2] = (B_left[i][1] + B_L2) * 0.5f;

		B_right[i][3] = B[3];
		B_right[i][2] = (B[2] + B[3]) * 0.5f;
		B_right[i][1] = (B_L2 + B_right[i][2]) * 0.5f;

		B_left[i][3] = (B_left[i][2] + B_right[i][1]) * 0.5f;
		B_right[i][0] = B_left[i][3];

		// Subdivide Inner curve I at t=0.5
		I_left[i][0] = I[0];
		I_left[i][1] = (I[0] + I[1]) * 0.5f;
		Vec3f I_L2 = (I[1] + I[2]) * 0.5f;
		I_left[i][2] = (I_left[i][1] + I_L2) * 0.5f;

		I_right[i][3] = I[3];
		I_right[i][2] = (I[2] + I[3]) * 0.5f;
		I_right[i][1] = (I_L2 + I_right[i][2]) * 0.5f;

		I_left[i][3] = (I_left[i][2] + I_right[i][1]) * 0.5f;
		I_right[i][0] = I_left[i][3];

		P3[i] = B_left[i][3];	// subdivision point on the boundary curve
		P2[i] = P3[i] * 2.0f - I_left[i][3]; // C1 reflection
		Q[i] = (P2[i] * 3.0f - P3[i]) * 0.5f; // auxiliary point
		P_center += Q[i];
	}

	P_center /= 3.0f;

	for (int i = 0; i < 3; i++)
		P1[i] = (Q[i] * 2.0f + P_center) / 3.0f;

	std::vector<VertexPosition> vertices;
	std::vector<VertexPosition> continuityVectorsVertices;
	vertices.reserve(m_vertexCount);
	continuityVectorsVertices.reserve(m_continuityVertexCount);

	for (int i = 0; i < 3; i++)
	{
		int prev = (i + 2) % 3;
		int next = (i + 1) % 3;

		// External Boundaries
		Vec3f P00 = corners[i]->m_position.ToVec3f();
		Vec3f P10 = B_left[i][1], P20 = B_left[i][2], P30 = P3[i];
		Vec3f P01 = B_right[prev][2], P02 = B_right[prev][1], P03 = P3[prev];

		// Internal Boundaries (Seams)
		Vec3f P33 = P_center;
		Vec3f P31 = P2[i], P32 = P1[i];
		Vec3f P13 = P2[prev], P23 = P1[prev];

		// A. External Twists (C1 between Bezier and Gregory sides)
		Vec3f P11u = P10 * 2.0f - I_left[i][1];
		Vec3f P21u = P20 * 2.0f - I_left[i][2];

		Vec3f P11v = P01 * 2.0f - I_right[prev][2];
		Vec3f P12v = P02 * 2.0f - I_right[prev][1];

		// B. Internal Seam Twists (C1 between Gregory sub-patches)
		Vec3f P21v = P20 + P31 - P30;
		Vec3f P12u = P02 + P13 - P03;

		// C. C1 at the central vertex
		//Vec3f P22 = P32 + P23 - P33;// P1[i] - P1[next] + P1[prev];
		Vec3f P22 = P1[i] - P1[next] + P1[prev];
		Vec3f P22u = P22;
		Vec3f P22v = P22;

		// The 20-Point patch sent to Domain Shader
		Vec3f patch[20] = {
			P00,  P10,  P20,  P30,  // 0-3
			P01,  P11u, P11v, P21u, P21v, P31, // 4-9
			P02,  P12u, P12v, P22u, P22v, P32, // 10-15
			P03,  P13,  P23,  P33   // 16-19
		};

		for (int v = 0; v < 20; v++)
			vertices.push_back({ patch[v].x, patch[v].y, patch[v].z });

		// Outward (Bezier side)
		continuityVectorsVertices.push_back(ToVertexPosition(P30));
		continuityVectorsVertices.push_back(ToVertexPosition(I_left[i][3]));
		// Inward (Gregory side)
		continuityVectorsVertices.push_back(ToVertexPosition(P30));
		continuityVectorsVertices.push_back(ToVertexPosition(P31));

		// 2. Corner Twists
		// V=0 Edge Outward and Inward
		continuityVectorsVertices.push_back(ToVertexPosition(P10));
		continuityVectorsVertices.push_back(ToVertexPosition(I_left[i][1]));
		continuityVectorsVertices.push_back(ToVertexPosition(P10));
		continuityVectorsVertices.push_back(ToVertexPosition(P11u));

		continuityVectorsVertices.push_back(ToVertexPosition(P20));
		continuityVectorsVertices.push_back(ToVertexPosition(I_left[i][2]));
		continuityVectorsVertices.push_back(ToVertexPosition(P20));
		continuityVectorsVertices.push_back(ToVertexPosition(P21u));

		// U=0 Edge Outward and Inward
		continuityVectorsVertices.push_back(ToVertexPosition(P01));
		continuityVectorsVertices.push_back(ToVertexPosition(I_right[prev][2]));
		continuityVectorsVertices.push_back(ToVertexPosition(P01));
		continuityVectorsVertices.push_back(ToVertexPosition(P11v));

		continuityVectorsVertices.push_back(ToVertexPosition(P02));
		continuityVectorsVertices.push_back(ToVertexPosition(I_right[prev][1]));
		continuityVectorsVertices.push_back(ToVertexPosition(P02));
		continuityVectorsVertices.push_back(ToVertexPosition(P12v));
	}


	device.UpdateBuffer(m_vertexBuffer, vertices.data(), vertices.size() * sizeof(VertexPosition));
	device.UpdateBuffer(m_continuityVertexBuffer, continuityVectorsVertices.data(), continuityVectorsVertices.size() * sizeof(VertexPosition));

	m_isDirty = false;
}

void GregoryPatch::ReplacePoint(Point* oldPoint, std::shared_ptr<Point> newPoint)
{
	bool replaced = false;
	for (int i = 0; i < 3; i++)
	{
		auto& edge = m_hole.edges[i];
		if (edge.startPoint == oldPoint)
		{
			edge.startPoint = newPoint.get();
			newPoint->m_surfaceLockCount++;
			replaced = true;
		}
		if (edge.endPoint == oldPoint)
		{
			edge.endPoint = newPoint.get();
			newPoint->m_surfaceLockCount++;
			replaced = true;
		}
		for (auto& ptWeak : edge.boundaryPoints)
		{
			if (auto pt = ptWeak.lock())
			{
				if (pt.get() == oldPoint)
				{
					ptWeak = newPoint;
					newPoint->m_surfaceLockCount++;
					replaced = true;
				}
			}
		}
		for (auto& ptWeak : edge.innerPoints)
		{
			if (auto pt = ptWeak.lock())
			{
				if (pt.get() == oldPoint)
				{
					ptWeak = newPoint;
					newPoint->m_surfaceLockCount++;
					replaced = true;
				}
			}
		}
	}
	if (replaced)
		MarkDirty();
}


