#pragma once
#include "structs.h"
#include "SceneObject.h"

struct RoughingParameters
{
	float cutterDiameter = 16.0f;    // [mm] K16 ball-nose cutter
	float cutterFluteHeight = 32.0f; // [mm] Max immersion depth = 2 * cutterDiameter

	float blockSizeX = 150.0f;       // [mm]
	float blockSizeY = 50.0f;        // [mm] Block height (Y in scene, Z in G-Code)
	float blockSizeZ = 150.0f;       // [mm]
	float baseHeight = 15.0f;        // [mm] Minimum base thickness

	int gridDivX = 600;              // Height map resolution X (0.25 mm at 600)
	int gridDivZ = 600;              // Height map resolution Z (0.25 mm at 600)

	float surfaceMargin = 0.5f;      // [mm] Roughing allowance left over model surfaces
	float baseMargin = 0.5f;         // [mm] Safety margin above baseHeight (above 15 mm base)
	float layer1Height = 33.0f;      // [mm] Minimum tip height for Layer 1 (2-layer roughing)
	float stepover = 7.5f;           // [mm] Distance between adjacent parallel roughing passes
	float safeHeight = 60.0f;        // [mm] Safe Z height in air above the 50 mm block

	float maxPlungeSlope = 4.0f;     // Max downward dY / dXZ slope (~14 deg from vertical)
	float simplifyTolerance = 0.03f; // [mm] Collinear / envelope simplification tolerance along scanlines
};

class PathGenerator
{
public:
	// Generates a 2-layer roughing toolpath (in simulator mm scene coordinates: x=X, y=Z_height, z=-Y)
	static std::vector<float3> GenerateRoughingPath(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const RoughingParameters& params = {});

	// Exports simulator mm scene waypoints to a G-Code file (applying X->X, -Z->Y, Y->Z mapping)
	static bool ExportGCode(
		const std::string& filepath,
		const std::vector<float3>& sceneMmWaypoints);
};
