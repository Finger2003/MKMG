#pragma once
#include "structs.h"
#include "DxDevice.h"

struct CutterParameters
{
	float diameter = 1.0f; // [mm]
	float height = 4.0f;   // [mm]
	bool isFlat = false;
};

class MillingSimulator
{
public:
	MillingSimulator(const DxDevice& device) : m_device(device) {}

	// Loads and parses G01 instructions from an ASCII file.
	// Returns true if at least one valid waypoint was loaded.
	bool LoadGCode(const std::string& filepath);

	// Advances the cutter along the loaded path independently of frame rate.
	void Update(float dt);

#pragma region Playback Controls
	void Start() { if (m_waypoints.size() >= 2) m_isRunning = true; }
	void Stop() { m_isRunning = false; }
	void Restart();
	void InstantComplete();
#pragma endregion


#pragma region Getters and Setters
	bool isRunning() const { return m_isRunning; }
	bool hasLoadedPath() const { return !m_waypoints.empty(); }

	void setSpeed(float speed) { m_speed = speed; }
	float getSpeed() const { return m_speed; }

	void setCutterParams(const CutterParameters& params) { m_cutter = params; }
	const CutterParameters& getCutterParams() const { return m_cutter; }

	const float3& getCutterPosition() const { return m_cutterPosition; }
	size_t getCurrentSegmentIndex() const { return m_currentSegment; }
	size_t getTotalSegments() const { return m_waypoints.empty() ? 0 : m_waypoints.size() - 1; }
	const std::vector<float3>& GetWaypoints() const { return m_waypoints; }

	const Microsoft::WRL::ComPtr<ID3D11Buffer>& getPathVertexBuffer() const { return m_pathVertexBuffer; }
	UINT getPathVertexCount() const { return static_cast<UINT>(m_pathVertices.size()); }
#pragma endregion
private:
	void MillSegment(const MathLib::Vec3f& startPos, const MathLib::Vec3f& endPos);
	void UpdatePathBuffer();
	const DxDevice& m_device;

	std::vector<float3> m_waypoints;
	float3 m_cutterPosition{ 0.0f, 0.0f, 0.0f };
	size_t m_currentSegment = 0;

	bool m_isRunning = false;
	float m_speed = 10.0f;
	CutterParameters m_cutter{};

	std::vector<VertexPosition> m_pathVertices;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_pathVertexBuffer;
	UINT m_pathBufferCapacity = 0;
	//UINT m_pathVertexCount = 0;
};

