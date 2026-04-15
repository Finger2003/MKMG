#include "pch.h"
#include "BSplineCurve.h"
#include "DxDevice.h"
#include "../MathLib/Vec3f.h"

using namespace std;
using namespace MathLib;

unsigned int BSplineCurve::s_nextId = 0;

BSplineCurve::BSplineCurve(std::vector<std::weak_ptr<Point>>&& controlPoints)
    : SceneObject("BSplineCurve" + to_string(s_nextId++), ObjectType::BSplineCurve), m_controlPoints(std::move(controlPoints))
{}

void BSplineCurve::CleanExpiredPoints()
{
    erase_if(m_controlPoints, [](const weak_ptr<Point>& wp) { return wp.expired(); });
}

void BSplineCurve::UpdatePolyline(const DxDevice& device)
{
    CleanExpiredPoints();

    if (m_controlPoints.size() < 2)
    {
        m_lineVertexCount = 0;
        m_curveVertexCount = 0;
        return;
    }

    bool needUpdate = false;
    if (m_lastPositions.size() != m_controlPoints.size())
        needUpdate = true;
    else
    {
        for (size_t i = 0; i < m_controlPoints.size(); ++i)
        {
            auto sp = m_controlPoints[i].lock();
            if (!sp ||
                sp->m_position.x != m_lastPositions[i].x ||
                sp->m_position.y != m_lastPositions[i].y ||
                sp->m_position.z != m_lastPositions[i].z)
            {
                needUpdate = true;
                break;
            }
        }
    }

    if (needUpdate)
    {
        m_lastPositions.clear();
        for (const auto& wp : m_controlPoints)
        {
            if (auto sp = wp.lock())
                m_lastPositions.push_back(sp->m_position);
        }

        size_t n = m_lastPositions.size();

        // 1. Control Polygon (Straight lines between De Boor points)
        std::vector<VertexPosition> lineVertices;
        lineVertices.reserve(n);
        for (size_t i = 0; i < n; i++)
            lineVertices.push_back({ m_lastPositions[i].x, m_lastPositions[i].y, m_lastPositions[i].z });

        // 2. Convert De Boor to Bernstein for the GS Pipeline
        std::vector<VertexPosition> vertices;

        std::vector<Vec3f> augPoints;
        augPoints.reserve(n + 2);
        Vec3f pFirst = m_lastPositions[0].ToVec3f();
        Vec3f pSecond = m_lastPositions[1].ToVec3f();
        augPoints.push_back(pFirst * 2.0f - pSecond);
        for (size_t i = 0; i < n; ++i)
            augPoints.push_back(m_lastPositions[i].ToVec3f());
        Vec3f pLast = m_lastPositions[n - 1].ToVec3f();
        Vec3f pPrev = m_lastPositions[n - 2].ToVec3f();
        augPoints.push_back(pLast * 2.0f - pPrev);
        size_t augN = augPoints.size();
        vertices.reserve((augN - 3) * 4);

        for (size_t i = 0; i <= augN - 4; ++i)
        {
            Vec3f p0 = augPoints[i];
            Vec3f p1 = augPoints[i + 1];
            Vec3f p2 = augPoints[i + 2];
            Vec3f p3 = augPoints[i + 3];

            // Convert B-Spline segment to Bezier segment
            Vec3f b0 = (p0 + p1 * 4.0f + p2) * (1.0f / 6.0f);
            Vec3f b1 = (p1 * 2.0f + p2) * (1.0f / 3.0f);
            Vec3f b2 = (p1 + p2 * 2.0f) * (1.0f / 3.0f);
            Vec3f b3 = (p1 + p2 * 4.0f + p3) * (1.0f / 6.0f);

            vertices.push_back({ b0.x, b0.y, b0.z });
            vertices.push_back({ b1.x, b1.y, b1.z });
            vertices.push_back({ b2.x, b2.y, b2.z });
            vertices.push_back({ b3.x, b3.y, b3.z });
        }

        //if (n == 2)
        //{
        //    // Degree elevation: Line (Degree 1) -> Cubic (Degree 3)
        //    Vec3f p0 = m_lastPositions[0].ToVec3f();
        //    Vec3f p1 = m_lastPositions[1].ToVec3f();

        //    Vec3f b0 = p0;
        //    Vec3f b1 = p0 * (2.0f / 3.0f) + p1 * (1.0f / 3.0f);
        //    Vec3f b2 = p0 * (1.0f / 3.0f) + p1 * (2.0f / 3.0f);
        //    Vec3f b3 = p1;

        //    vertices.push_back({ b0.x, b0.y, b0.z });
        //    vertices.push_back({ b1.x, b1.y, b1.z });
        //    vertices.push_back({ b2.x, b2.y, b2.z });
        //    vertices.push_back({ b3.x, b3.y, b3.z });
        //}
        //else if (n == 3)
        //{
        //    // Degree elevation: Quadratic Bezier (Degree 2) -> Cubic (Degree 3)
        //    Vec3f p0 = m_lastPositions[0].ToVec3f();
        //    Vec3f p1 = m_lastPositions[1].ToVec3f();
        //    Vec3f p2 = m_lastPositions[2].ToVec3f();

        //    Vec3f b0 = p0;
        //    Vec3f b1 = p0 * (1.0f / 3.0f) + p1 * (2.0f / 3.0f);
        //    Vec3f b2 = p1 * (2.0f / 3.0f) + p2 * (1.0f / 3.0f);
        //    Vec3f b3 = p2;

        //    vertices.push_back({ b0.x, b0.y, b0.z });
        //    vertices.push_back({ b1.x, b1.y, b1.z });
        //    vertices.push_back({ b2.x, b2.y, b2.z });
        //    vertices.push_back({ b3.x, b3.y, b3.z });
        //}
        //else if (n >= 4)
        //{
        //    // Standard C2 B-Spline segments
        //    vertices.reserve((n - 3) * 4);
        //    for (size_t i = 0; i <= n - 4; ++i)
        //    {
        //        Vec3f p0 = m_lastPositions[i].ToVec3f();
        //        Vec3f p1 = m_lastPositions[i + 1].ToVec3f();
        //        Vec3f p2 = m_lastPositions[i + 2].ToVec3f();
        //        Vec3f p3 = m_lastPositions[i + 3].ToVec3f();

        //        // Convert B-Spline segment to Bezier segment
        //        Vec3f b0 = (p0 + p1 * 4.0f + p2) * (1.0f / 6.0f);
        //        Vec3f b1 = (p1 * 2.0f + p2) * (1.0f / 3.0f);
        //        Vec3f b2 = (p1 + p2 * 2.0f) * (1.0f / 3.0f);
        //        Vec3f b3 = (p1 + p2 * 4.0f + p3) * (1.0f / 6.0f);

        //        vertices.push_back({ b0.x, b0.y, b0.z });
        //        vertices.push_back({ b1.x, b1.y, b1.z });
        //        vertices.push_back({ b2.x, b2.y, b2.z });
        //        vertices.push_back({ b3.x, b3.y, b3.z });
        //    }
        //}
        m_lineVertexCount = static_cast<UINT>(lineVertices.size());
        if (lineVertices.size() >= 2)
            UpdateDynamicBuffer(device, m_lineVertexBuffer, m_lineBufferCapacity, lineVertices);
        else
            m_lineVertexCount = 0;

        m_curveVertexCount = static_cast<UINT>(vertices.size());
        if (vertices.size() >= 4)
            UpdateDynamicBuffer(device, m_curveVertexBuffer, m_curveBufferCapacity, vertices);
        else
            m_curveVertexCount = 0;
    }
}

void BSplineCurve::UpdateDynamicBuffer(const DxDevice& device, Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, UINT& capacity, const std::vector<VertexPosition>& data)
{
    UINT requiredCount = static_cast<UINT>(data.size());
    if (requiredCount > capacity)
    {
        capacity = std::max({ requiredCount, static_cast<UINT>(capacity * 1.5), 16u });
        buffer = device.CreateDynamicVertexBuffer<VertexPosition>(capacity);
    }
    device.UpdateBuffer(buffer, data.data(), requiredCount * sizeof(VertexPosition));
}