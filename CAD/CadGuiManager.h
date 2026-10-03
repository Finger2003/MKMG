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
	bool UseCursorAsHint() const { return m_useCursorAsHint; }
	bool IsSurfacePreviewActive() const { return m_showSurfacePopup; }
	void ShowIntersectionPopup() { m_showIntersectionPopup = true; }
	float GetIntersectionStep() const { return m_intersectionStep; }

private:
	CadApplication& m_app;

#pragma region SurfaceCreation
	int m_previewSegU = 1;
	int m_previewSegV = 1;
	float m_previewWidth = 1.0f; // Width
	float m_previewDim2 = 1.0f; // Length or Height
	float m_previewRadius = m_previewWidth / (2.0f * std::numbers::pi_v<float>);
	int m_previewType = 0; // 0 = C0, 1 = C2
	int m_previewShape = 0;     // 0 = Flat, 1 = Cylinder
	bool m_showSurfacePopup = false;
#pragma endregion


#pragma region Intersections
	bool m_showIntersectionPopup = false;
	bool m_useCursorAsHint = false;
	float m_intersectionStep = 0.01f;
	int m_previewSurfaceIndex = 0;
	std::weak_ptr<Intersection> m_previewIntersection;
#pragma endregion

	char m_renameBuffer[128] = {};
	char m_filterBuffer[128] = {};
	std::optional<size_t> m_nameEditingIndex = std::nullopt;

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