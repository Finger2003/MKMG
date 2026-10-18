#include "pch.h"
#include "MillingSimulator.h"
#include "../MathLib/Vec3f.h"

bool MillingSimulator::LoadGCode(const std::string& filepath)
{
	std::ifstream file(filepath);
	if (!file.is_open())
		return false;

	std::vector<float3> parsedWaypoints;
	float3 currentPos{ 0.0f, 0.0f, 0.0f };

	std::string line;
	while (std::getline(file, line))
	{
		std::string_view view(line);

		// Skip empty lines
		size_t firstValid = view.find_first_not_of(" \t\r\n");
		if (firstValid == std::string_view::npos)
			continue;

		view.remove_prefix(firstValid);

		// Ignore comments
		if (view.front() == '%' || view.front() == '/' || view.front() == ';')
			continue;

		// Look for G01 command
		size_t g01Pos = view.find("G01");
		if (g01Pos == std::string::npos)
			continue;

		// Move past "G01"
		view.remove_prefix(g01Pos + 3);

		bool hasCoordinate = false;
		while (!view.empty())
		{
			// Find the next axis letter (X, Y, or Z)
			size_t axisPos = view.find_first_of("XYZxyz");

			// No more axis letters found, break the loop
			if (axisPos == std::string_view::npos)
				break;

			char axis = static_cast<char>(std::toupper(view[axisPos]));
			view.remove_prefix(axisPos + 1);

			// Skip whitespace after the axis letter
			size_t numStart = view.find_first_not_of(" \t\r\n");
			if (numStart == std::string_view::npos)
				break;

			view.remove_prefix(numStart);

			float val = 0.0f;
			auto [ptr, ec] = std::from_chars(view.data(), view.data() + view.size(), val);

			if (ec == std::errc{})
			{
				if (axis == 'X') currentPos.x = val;
				else if (axis == 'Y') currentPos.y = val;
				else if (axis == 'Z') currentPos.z = val;

				hasCoordinate = true;
				view.remove_prefix(ptr - view.data());
			}
		}

		if (hasCoordinate)
			parsedWaypoints.push_back(currentPos);
	}

	if (parsedWaypoints.empty())
		return false;
	m_waypoints = std::move(parsedWaypoints);

	size_t size = m_waypoints.size();
	UINT requiredCapacity = static_cast<UINT>(size);
	m_pathVertices.reserve(size);

	if (!m_pathVertexBuffer || m_pathBufferCapacity < requiredCapacity)
	{
		m_pathBufferCapacity = requiredCapacity;
		m_pathVertexBuffer = m_device.CreateDynamicVertexBuffer<VertexPosition>(m_pathBufferCapacity);
	}

	Restart();
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

	UpdatePathBuffer();
}

void MillingSimulator::Restart()
{
	m_isRunning = false;
	m_currentSegment = 0;
	m_cutterPosition = m_waypoints.empty() ? float3{ 0.0f, 0.0f, 0.0f } : m_waypoints.front();
	m_pathVertices.clear();
	UpdatePathBuffer();
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
	UpdatePathBuffer();
}

void MillingSimulator::MillSegment(const MathLib::Vec3f& startPos, const MathLib::Vec3f& endPos)
{}

void MillingSimulator::UpdatePathBuffer()
{
	if (m_waypoints.size() < 2)
	{
		m_pathVertices.clear();
		return;
	}

	// Remove the previous frame's temporary cutter position
	if (!m_pathVertices.empty())
		m_pathVertices.pop_back();

	// Append any new waypoints that have been reached since the last update
	while (m_pathVertices.size() <= m_currentSegment)
	{
		const float3& wp = m_waypoints[m_pathVertices.size()];
		m_pathVertices.push_back({ wp.x, wp.y, wp.z });
	}

	// Append the current in-progress cutter position
	bool hasInProgressSegment = (m_currentSegment + 1 < m_waypoints.size());
	if (hasInProgressSegment)
		m_pathVertices.push_back({ m_cutterPosition.x, m_cutterPosition.y, m_cutterPosition.z });

	// Update the GPU buffer
	m_device.UpdateBuffer(m_pathVertexBuffer, m_pathVertices.data(), m_pathVertices.size() * sizeof(VertexPosition));
}
