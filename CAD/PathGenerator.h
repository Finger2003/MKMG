#pragma once
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
	float simplifyTolerance = 0.15f; // [mm] Collinear / envelope simplification tolerance along scanlines

	SceneObject* baseSurface = nullptr; // Optional designated flat base plane surface (uses baseHeight + baseMargin)
};

struct BaseFinishingParameters
{
	float cutterDiameter = 10.0f;       // [mm] F10 flat-end cutter (R = 5 mm)
	float cutterFluteHeight = 20.0f;    // [mm] Max immersion depth = 2 * cutterDiameter

	float blockSizeX = 150.0f;          // [mm]
	float blockSizeY = 50.0f;           // [mm] Block height (Y in scene, Z in G-Code)
	float blockSizeZ = 150.0f;          // [mm]
	float baseHeight = 15.0f;           // [mm] Minimum base thickness (height of the flat base)

	float contourMargin = 1.0f;         // [mm] Extra margin left by zigzag around contour, cleared by final contour pass
	float stepover = 9.0f;              // [mm] Zigzag pass spacing (<= 2R - eps, where 2R = 10 mm)
	float safeHeight = 60.0f;           // [mm] Safe Z height in air above the 50 mm block

	SceneObject* baseSurface = nullptr; // Optional designated flat base plane surface (if null, uses baseHeight)
	std::string seedsJsonPath;          // Optional JSON file with per-surface cursorSeeds for base CL intersections
};

class PathGenerator
{
public:
	// Generates a 2-layer roughing toolpath (in simulator mm scene coordinates: x=X, y=Z_height, z=-Y)
	static std::vector<float3> GenerateRoughingPath(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const RoughingParameters& params = {});

	// Generates a flat base finishing + outer contour toolpath for F10 cutter
	static std::vector<float3> GenerateBaseFinishingPath(
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		const BaseFinishingParameters& params = {});

	// Exports simulator mm scene waypoints to a G-Code file (applying X->X, -Z->Y, Y->Z mapping)
	static bool ExportGCode(
		const std::string& filepath,
		const std::vector<float3>& sceneMmWaypoints);
};
