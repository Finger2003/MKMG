#pragma once

class CadApplication;
struct Curve;
struct Surface;
struct GregoryPatch;
struct Torus;
struct Point;
struct Intersection;

class CadGuiManager
{
public:
	explicit CadGuiManager(CadApplication& app) : m_app(app) {}
	~CadGuiManager() = default;

	void DrawMenu();

private:
	CadApplication& m_app;
	char m_renameBuffer[128] = {};
	char m_filterBuffer[128] = {};

	void DrawActionCombo();
	void DrawListMenu(int selectedCount, int selectedPoints, std::shared_ptr<Curve> activeCurve);
	void DrawCurveList(Curve* curve, int selectedCount);
	void DrawSurfaceList(Surface* surface, int selectedCount);
	void DrawEditMenu();
	void DrawTorusMenu(Torus& torus);
	void DrawPointMenu(Point& selectedObj);
	void DrawSurfaceMenu(Surface& surface);
	void DrawPatchMenu(GregoryPatch& patch);
	void DrawEditGroupMenu(int selectedCount);
	void DrawCameraSettingsMenu();
	void DrawCursorSettingsMenu();
	void DrawIntersectionList(std::shared_ptr<Intersection> intersection);
};