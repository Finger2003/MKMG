#include "pch.h"
#include "MillingSimulator.h"
#include "../MathLib/Vec3f.h"


namespace
{
	enum MillingErrorFlags : uint32_t
	{
		Error_None = 0,
		Error_NonCuttingPart = 1 << 0,
		Error_BelowBaseHeight = 1 << 1,
		Error_VerticalDownward = 1 << 2
	};

	float3 SceneToGCodeCoords(const float3& scenePos)
	{
		// Inverse of G-Code mapping (X -> x, Y -> -z, Z -> y)
		return { scenePos.x, -scenePos.z, scenePos.y };
	}

	constexpr float cVerticalAngleToleranceDeg = 1.5f;
}

bool MillingSimulator::LoadGCode(const std::string& filepath)
{
	std::ifstream file(filepath);
	if (!file.is_open())
		return false;

	std::vector<float3> parsedWaypoints;
	std::vector<size_t> parsedLines;
	float3 currentPos{ 0.0f, 0.0f, 0.0f };

	std::string line;
	size_t lineNumber = 0;
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
				else if (axis == 'Y') currentPos.z = -val;
				else if (axis == 'Z') currentPos.y = val;

				hasCoordinate = true;
				view.remove_prefix(ptr - view.data());
			}
		}

		if (hasCoordinate)
		{
			parsedWaypoints.push_back(currentPos);
			parsedLines.push_back(lineNumber);
		}
		lineNumber++;
	}

	if (parsedWaypoints.empty())
		return false;

	m_waypoints = std::move(parsedWaypoints);
	m_waypointLines = std::move(parsedLines);
	m_loadedFilePath = filepath;
	m_dumpFilePath = filepath + "_dump.txt";
	m_hasWrittenErrorToDump = false;
	m_pendingError.reset();

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
			m_reportedSegmentErrors = 0;
			continue;
		}

		if (distanceToTravel >= distToTarget)
		{
			m_cutterPosition = targetPos;
			//MillSegment(cutterPosVec, targetPosVec);
			
			if (!MillSegment(cutterPosVec, targetPosVec))
				break;

			distanceToTravel -= distToTarget;
			m_currentSegment++;
			m_reportedSegmentErrors = 0;
		}
		else
		{
			MathLib::Vec3f dir = toTarget / distToTarget;
			MathLib::Vec3f nextPos = cutterPosVec + dir * distanceToTravel;
			//MillSegment(cutterPosVec, nextPos);

			if (!MillSegment(cutterPosVec, nextPos))
				break;

			m_cutterPosition = nextPos;
			distanceToTravel = 0.0f;
		}

		if (m_currentSegment + 1 >= m_waypoints.size())
			m_isRunning = false;
	}

	UpdatePathBuffer();
	UpdateBlockBuffer();
}

void MillingSimulator::InitBlock(const BlockParameters& params)
{
	m_block = params;
	m_block.divX = std::max(2, m_block.divX);
	m_block.divZ = std::max(2, m_block.divZ);

	const int topCols = m_block.divX + 1;
	const int topRows = m_block.divZ + 1;
	const int totalCols = topCols + 2;	// +2 for the border vertices
	const int totalRows = topRows + 2;	// +2 for the border vertices

	m_blockVertices.resize(static_cast<size_t>(totalCols) * totalRows);

	const float halfX = m_block.sizeX * 0.5f;
	const float halfZ = m_block.sizeZ * 0.5f;
	const float dx = m_block.sizeX / m_block.divX;
	const float dz = m_block.sizeZ / m_block.divZ;

	for (int rz = 0; rz < totalRows; rz++)
	{
		int clampedIz = std::clamp(rz - 1, 0, m_block.divZ);
		float z = -halfZ + clampedIz * dz;
		bool isBorderZ = (rz == 0 || rz == totalRows - 1);

		for (int cx = 0; cx < totalCols; cx++)
		{
			int clampedIx = std::clamp(cx - 1, 0, m_block.divX);
			float x = -halfX + clampedIx * dx;
			bool isBorderX = (cx == 0 || cx == totalCols - 1);

			float y = (isBorderX || isBorderZ) ? 0.0f : m_block.sizeY;
			m_blockVertices[static_cast<size_t>(rz) * totalCols + cx] = { x, y, z };
		}
	}


	std::vector<uint32_t> indices;
	indices.reserve(static_cast<size_t>(totalRows - 1) * static_cast<size_t>(totalCols - 1) * 6 + 6);
	for (int rz = 0; rz < totalRows - 1; ++rz)
	{
		for (int cx = 0; cx < totalCols - 1; ++cx)
		{
			uint32_t i00 = static_cast<uint32_t>(rz * totalCols + cx);
			uint32_t i10 = i00 + 1;
			uint32_t i01 = static_cast<uint32_t>((rz + 1) * totalCols + cx);
			uint32_t i11 = i01 + 1;

			indices.push_back(i00);
			indices.push_back(i10);
			indices.push_back(i01);

			indices.push_back(i10);
			indices.push_back(i11);
			indices.push_back(i01);
		}
	}

	// Bottom cap
	uint32_t c00 = 0;
	uint32_t c10 = static_cast<uint32_t>(totalCols - 1);
	uint32_t c01 = static_cast<uint32_t>((totalRows - 1) * totalCols);
	uint32_t c11 = c01 + c10;
	indices.push_back(c00); indices.push_back(c01); indices.push_back(c10);
	indices.push_back(c10); indices.push_back(c01); indices.push_back(c11);

	m_blockIndexCount = static_cast<UINT>(indices.size());
	m_blockVertexBuffer = m_device.CreateDynamicVertexBuffer(m_blockVertices);
	m_blockIndexBuffer = m_device.CreateIndexBuffer(indices);
	m_isBlockDirty = false;
}

void MillingSimulator::Restart()
{
	m_reportedSegmentErrors = 0;
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
		MathLib::Vec3f startVec = m_cutterPosition.ToVec3f();
		MathLib::Vec3f targetVec = targetPos.ToVec3f();
		m_cutterPosition = targetPos;

		if (!MillSegment(startVec, targetVec))
			break;

		m_currentSegment++;
		m_reportedSegmentErrors = 0;
	}
	m_isRunning = false;
	UpdatePathBuffer();
	UpdateBlockBuffer();
}

bool MillingSimulator::MillSegment(const MathLib::Vec3f& startPos, const MathLib::Vec3f& endPos)
{
	const float R = m_cutter.diameter * 0.5f;
	if (R <= 0.0f || m_blockVertices.empty())
		return true;

	// Early exit if the segment is completely above the block's top surface
	if (std::min(startPos.y, endPos.y) >= m_block.sizeY)
		return true;

	const float R2 = R * R;
	const float halfX = m_block.sizeX * 0.5f;
	const float halfZ = m_block.sizeZ * 0.5f;
	const float dx = m_block.sizeX / m_block.divX;
	const float dz = m_block.sizeZ / m_block.divZ;
	const float invDx = 1.0f / dx;
	const float invDz = 1.0f / dz;

	// Clamped 2D bounding box of the segment extended by the cutter radius
	float minX = std::min(startPos.x, endPos.x) - R;
	float maxX = std::max(startPos.x, endPos.x) + R;
	float minZ = std::min(startPos.z, endPos.z) - R;
	float maxZ = std::max(startPos.z, endPos.z) + R;

	int ixMin = std::max(0, static_cast<int>(std::ceil((minX + halfX) * invDx)));
	int ixMax = std::min(m_block.divX, static_cast<int>(std::floor((maxX + halfX) * invDx)));
	int izMin = std::max(0, static_cast<int>(std::ceil((minZ + halfZ) * invDz)));
	int izMax = std::min(m_block.divZ, static_cast<int>(std::floor((maxZ + halfZ) * invDz)));
	if (ixMin > ixMax || izMin > izMax)
		return true;

	MathLib::Vec3f posDelta = endPos - startPos;
	const float ux = posDelta.x;
	const float uz = posDelta.z;
	const float dy = posDelta.y;
	const float lenXZSq = ux * ux + uz * uz;
	const float lenXZ = std::sqrt(lenXZSq);
	const bool isHorizontal = (std::abs(dy) <= 1e-6f);
	const bool isVerticalPlunge = (lenXZSq <= 1e-8f);

	constexpr float cVertTolRad = cVerticalAngleToleranceDeg * (std::numbers::pi_v<float> / 180.0f);
	const float maxAllowedTan = std::tan(cVertTolRad);
	const bool isSteepDownwardMove = (dy < -1e-6f) && (lenXZ <= (-dy) * maxAllowedTan);

	const float invLenXZSq = isVerticalPlunge ? 0.0f : (1.0f / lenXZSq);
	const float invLenXZ = std::sqrt(invLenXZSq);
	const float slopeFactor = dy / std::sqrt(lenXZSq + dy * dy);

	const int totalCols = m_block.divX + 3; // (divX + 1) + 2 skirt columns
	constexpr float cHeightEps = 1e-4f;
	uint32_t detectedErrors = Error_None;

	for (int iz = izMin; iz <= izMax; iz++)
	{
		const float pz = -halfZ + iz * dz;
		const float vz = pz - startPos.z;
		const size_t rowOffset = static_cast<size_t>(iz + 1) * totalCols; // +1 for skirt row

		for (int ix = ixMin; ix <= ixMax; ix++)
		{
			VertexPosition& vert = m_blockVertices[rowOffset + (static_cast<size_t>(ix) + 1)]; // +1 for skirt col

			const float px = -halfX + ix * dx;
			const float vx = px - startPos.x;

			float cutHeight = m_block.sizeY;
			float entryTipY = startPos.y;
			//float tipY = startPos.y;

			if (isVerticalPlunge)
			{
				float distSq = vx * vx + vz * vz;
				if (distSq > R2)
					continue;

				float minY = std::min(startPos.y, endPos.y);
				entryTipY = startPos.y;
				cutHeight = m_cutter.isFlat
					? minY
					: (minY + R - std::sqrt(std::max(0.0f, R2 - distSq)));
			}
			else
			{
				// Unclamped projection parameter t along the 2D line
				float t = (vx * ux + vz * uz) * invLenXZSq;
				if (isHorizontal)
				{
					float tBest = std::clamp(t, 0.0f, 1.0f);
					float qx = ux * tBest - vx;
					float qz = uz * tBest - vz;
					float distSq = qx * qx + qz * qz;
					if (distSq > R2)
						continue;

					entryTipY = startPos.y;
					cutHeight = m_cutter.isFlat
						? startPos.y
						: (startPos.y + R - std::sqrt(R2 - distSq));
				}
				else
				{
					// Perpendicular distance squared to the infinite 2D line using extrapolated t
					float hx = ux * t - vx;
					float hz = uz * t - vz;
					float hSq = hx * hx + hz * hz;

					if (hSq > R2)
						continue;

					float dt = std::sqrt(R2 - hSq) * invLenXZ;

					float tEnter = std::max(0.0f, t - dt);
					float tExit = std::min(1.0f, t + dt);
					if (tEnter > tExit)
						continue;

					entryTipY = startPos.y + tEnter * dy;
					if (m_cutter.isFlat)
					{
						float tBest = (dy >= 0.0f) ? tEnter : tExit;
						cutHeight = startPos.y + tBest * dy;
					}
					else
					{
						float tBest = std::clamp(t - slopeFactor * dt, 0.0f, 1.0f);
						float qx = ux * tBest - vx;
						float qz = uz * tBest - vz;
						float distSq = qx * qx + qz * qz;
						cutHeight = startPos.y + tBest * dy + R - std::sqrt(std::max(0.0f, R2 - distSq));
					}
				}
			}

			if (cutHeight < vert.y - cHeightEps)
			{
				// 1. Check milling with non-cutting part of the tool
				if (vert.y - entryTipY > m_cutter.height + cHeightEps)
					detectedErrors |= Error_NonCuttingPart;

				// 2. Check immersion below the minimum base height
				if (cutHeight < m_block.baseHeight - cHeightEps)
					detectedErrors |= Error_BelowBaseHeight;

				// 3. Check vertical / near-vertical downward milling
				if (isSteepDownwardMove)
					detectedErrors |= Error_VerticalDownward;

				vert.y = cutHeight;
				m_isBlockDirty = true;
			}
		}
	}

	uint32_t newErrors = detectedErrors & ~m_reportedSegmentErrors;
	if (newErrors != Error_None)
	{
		m_reportedSegmentErrors |= newErrors;
		ReportMillingError(newErrors, endPos);
		if (m_pauseOnError)
			return false;
	}

	return true;
}

void MillingSimulator::ReportMillingError(uint32_t errorFlags, const MathLib::Vec3f& errorScenePos)
{
	if (m_pauseOnError)
		m_isRunning = false;

	size_t cmdIdx = std::min(m_currentSegment + 1, m_waypoints.size() - 1);
	size_t startIdx = std::min(m_currentSegment, m_waypoints.size() - 1);

	MillingErrorInfo info{
		.lineNumber = (cmdIdx < m_waypointLines.size()) ? m_waypointLines[cmdIdx] : 0,
		.moveStartGCode = SceneToGCodeCoords(m_waypoints[startIdx]),
		.moveEndGCode = SceneToGCodeCoords(m_waypoints[cmdIdx]),
		.errorPosGCode = SceneToGCodeCoords({ errorScenePos.x, errorScenePos.y, errorScenePos.z }),
		.dumpFilePath = m_dumpFilePath
	};

	std::ostringstream desc;
	if (errorFlags & Error_NonCuttingPart)
	{
		desc << "- Milling with non-cutting part of the tool (cut depth exceeds cutter flute height of "
			<< std::fixed << std::setprecision(2) << m_cutter.height << " mm).\n";
	}
	if (errorFlags & Error_BelowBaseHeight)
	{
		desc << "- Tool immersed too deeply into the base (cut height below minimum base height of "
			<< std::fixed << std::setprecision(2) << m_block.baseHeight << " mm).\n";
	}
	if (errorFlags & Error_VerticalDownward)
	{
		desc << "- Vertical downward milling detected (plunge angle < "
			<< std::fixed << std::setprecision(1) << cVerticalAngleToleranceDeg
			<< " deg from vertical).\n";
	}
	info.errorDescription = desc.str();

	if (!m_dumpFilePath.empty())
	{
		std::ios_base::openmode mode = m_hasWrittenErrorToDump
			? (std::ios::out | std::ios::app)
			: (std::ios::out | std::ios::trunc);

		std::ofstream dumpFile(m_dumpFilePath, mode);
		if (dumpFile.is_open())
		{
			m_hasWrittenErrorToDump = true;
			dumpFile << "============================================================\n";
			dumpFile << "MILLING SIMULATION ERROR REPORT\n";
			dumpFile << "============================================================\n";
			dumpFile << "Source G-Code File : " << m_loadedFilePath << "\n";
			dumpFile << "Original Line No.  : " << info.lineNumber << "\n";
			dumpFile << std::fixed << std::setprecision(3);
			dumpFile << "Move Start (G-Code): X=" << info.moveStartGCode.x
				<< " Y=" << info.moveStartGCode.y
				<< " Z=" << info.moveStartGCode.z << "\n";
			dumpFile << "Move End   (G-Code): X=" << info.moveEndGCode.x
				<< " Y=" << info.moveEndGCode.y
				<< " Z=" << info.moveEndGCode.z << "\n";
			dumpFile << "Stopped At (G-Code): X=" << info.errorPosGCode.x
				<< " Y=" << info.errorPosGCode.y
				<< " Z=" << info.errorPosGCode.z << "\n";
			dumpFile << "Error Type(s)      :\n" << info.errorDescription;
			dumpFile << "============================================================\n\n";
		}
	}

	m_pendingError = std::move(info);
}

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

void MillingSimulator::UpdateBlockBuffer()
{
	if (!m_isBlockDirty || !m_blockVertexBuffer)
		return;

	m_device.UpdateBuffer(m_blockVertexBuffer, m_blockVertices.data(), m_blockVertices.size() * sizeof(VertexPosition));
	m_isBlockDirty = false;
}
