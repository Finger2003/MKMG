#include "pch.h"
#include "MillingSimulator.h"
#include "../MathLib/Vec3f.h"

bool MillingSimulator::LoadGCode(const std::string& filepath)
{
	std::ifstream file(filepath);
	if (!file.is_open())
		return false;

	std::vector<float3> parsedWaypoints;
	float3 currentPost{ 0.0f, 0.0f, 0.0f };

	std::string line;
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '%' || line[0] == '/')
			continue;

		size_t g01Pos = line.find("G01");
		if (g01Pos == std::string::npos)
			continue;

		size_t i = g01Pos + 3;
		bool hasCoordinate = false;
		while (i < line.size())
		{
			// Skip whitespace before the coordinate
			while (i < line.size() && std::isspace(line[i]))
				i++;

			if (i >= line.size())
				break;

			char axis = std::toupper(line[i++]);
			if (axis != 'X' && axis != 'Y' && axis != 'Z')
				continue;

			// Skip whitespace between axis and the number
			while (i < line.size() && std::isspace(line[i]))
				i++;

			// Parse the floating-point number after the axis letter
			float val = 0.0f;
			const char* start = line.data() + i;
			const char* end = line.data() + line.size();

			auto [ptr, ec] = std::from_chars(start, end, val);
			if (ec == std::errc{})
			{
				if (axis == 'X') currentPost.x = val;
				else if (axis == 'Y') currentPost.y = val;
				else if (axis == 'Z') currentPost.z = val;

				hasCoordinate = true;

				// Move the index to the position after the parsed number
				i = static_cast<size_t>(ptr - line.data());
			}
		}

		if (hasCoordinate)
			parsedWaypoints.push_back(currentPost);
	}

	if (parsedWaypoints.empty())
		return false;
	m_waypoints = std::move(parsedWaypoints);
	return true;
}

void MillingSimulator::Update(float dt)
{
	if (!m_isRunning || m_waypoints.size() < 2)
		return;

	float distanceToTravel = m_speed * dt;

	while (distanceToTravel > 0.0f && m_currentSegment + 1 < m_waypoints.size())
	{
		const float3& targetPos = m_waypoints[m_currentSegment + 1];
		MathLib::Vec3f targetPosVec = targetPos.ToVec3f();
		MathLib::Vec3f cutterPosVec = m_cutterPosition.ToVec3f();
		MathLib::Vec3f toTarget = targetPosVec - cutterPosVec;
		float distToTarget = toTarget.length();

		if (distToTarget <= 1e-6f)
		{
			m_cutterPosition = targetPos;
			m_currentSegment++;
			continue;
		}

		if (distanceToTravel >= distToTarget)
		{
			MillSegment(cutterPosVec, targetPosVec);
			m_cutterPosition = targetPos;
			distanceToTravel -= distToTarget;
			m_currentSegment++;
		}
		else
		{
			MathLib::Vec3f dir = toTarget / distToTarget;
			MathLib::Vec3f nextPos = cutterPosVec + dir * distanceToTravel;
			MillSegment(cutterPosVec, nextPos);
			m_cutterPosition = nextPos;
			distanceToTravel = 0.0f;
		}

		if (m_currentSegment + 1 >= m_waypoints.size())
			m_isRunning = false;
	}
}

void MillingSimulator::Restart()
{
	m_isRunning = false;
	m_currentSegment = 0;
	m_cutterPosition = m_waypoints.empty() ? float3{ 0.0f, 0.0f, 0.0f } : m_waypoints.front();
}

void MillingSimulator::InstantComplete()
{
	if (m_waypoints.size() < 2)
		return;

	while (m_currentSegment + 1 < m_waypoints.size())
	{
		const float3& targetPos = m_waypoints[m_currentSegment + 1];
		MillSegment(m_cutterPosition.ToVec3f(), targetPos.ToVec3f());
		m_cutterPosition = targetPos;
		m_currentSegment++;
	}
	m_isRunning = false;
}

void MillingSimulator::MillSegment(const MathLib::Vec3f& startPos, const MathLib::Vec3f& endPos)
{}
