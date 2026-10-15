#pragma once
#include "DxApplication.h"
#include "../MathLib/Mat4f.h"
#include "../MathLib/Vec3f.h"
#include "SceneObject.h"
#include "Torus.h"
#include "Point.h"
#include "BezierCurve.h"
#include "BSplineCurve.h"
#include "InterpolatingCurve.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
#include "Cursor3D.h"
#include "Camera.h"
#include "SurfaceBuilder.h"
#include "GregoryPatch.h"
#include "Intersection.h"
#include "CadGuiManager.h"
#include "CamGuiManager.h"
#include "SceneRenderer.h"
#include "Clock.h"
#include "MillingSimulator.h"

enum class AppMode
{
	CAD_Modeling,
	CAM_Simulation
};

enum class InteractionMode
{
	None,
	Orbiting,
	Panning
};

enum class MenuState
{
	List,
	Edit,
	EditGroup
};
enum class EditAction
{
	None,
	TranslateFree, TranslateX, TranslateY, TranslateZ,
	RotateFree, RotateX, RotateY, RotateZ,
	Scale, ScaleX, ScaleY, ScaleZ
};


template<typename T>
concept IsFileDialog = std::derived_from<T, IFileDialog>;

template<IsFileDialog T>
struct DialogTraits;
template<>
struct DialogTraits<IFileOpenDialog>
{
	static constexpr const CLSID& Clsid = CLSID_FileOpenDialog;
};
template<>
struct DialogTraits<IFileSaveDialog>
{
	static constexpr const CLSID& Clsid = CLSID_FileSaveDialog;
};

class CadApplication : public DxApplication
{
	/**
	 * @brief Creates application instance.
	 *
	 * Creates new application instance with a single window of given width, height and title.
	 *
	 * @param [in] hInstance Application instance handle (passed to WinMain by the system).
	 * @param [in] wndWidth Desired window width.
	 * @param [in] wndHeight Desired window height.
	 * @param [in] wndTitle Desired window title.
	 */
public:
	explicit CadApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"CADApp");

	/**
	 * @brief Handles system messages received by the window.
	 *
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override;

	void HandleCameraInteraction(int xPos, int yPos);


	virtual ~CadApplication();

	/**
	 * @brief Updates application resources that depend on the size of the window.
	 *
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	void UpdateResources(int width, int height) override;


	AppMode getAppMode() const { return m_appMode; }
	void setAppMode(AppMode mode) { m_appMode = mode; }
	HWND getWindowHandle() const { return m_window.getHandle(); }

	MillingSimulator& getMillingSimulator() { return m_millingSimulator; }
	const MillingSimulator& getMillingSimulator() const { return m_millingSimulator; }
protected:
	void Render() override; // Renders the scene to the window.


	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> m_depthBuffer;
	SceneRenderer m_sceneRenderer;

private:
	AppMode m_appMode = AppMode::CAD_Modeling;

#pragma region Constants
	static constexpr float cMenuWidth = 400.0f;
	static constexpr int cMinWidth = 500;
	static constexpr int cMinHeight = 500;
#pragma endregion
	Camera m_camera;

	MathLib::Vec3f m_startArcballVector{};
	std::vector<std::shared_ptr<SceneObject>> m_sceneObjects;

	InteractionMode m_interactionMode = InteractionMode::None;
	POINT m_lastMousePos{};
	POINT m_startMousePos{};
	SIZE m_renderSize{};

	float3 m_cursorPosition{ 0.0f, 0.0f, 0.0f };

	std::optional<size_t> m_lastClickedIndex = std::nullopt;
	std::optional<size_t> m_lastCurveClickedIndex = std::nullopt;

	MenuState m_menuState = MenuState::List;
	EditAction m_currentEditAction = EditAction::None;

	float m_editAnchorDepth = 0.0f;
	float m_editObjScreenX, m_editObjScreenY;
	float3 m_groupEditCenter;

	mutable std::optional<float3> m_selectionCenterCache = std::nullopt;

	mutable bool m_selectionDirty = true;
	bool m_isEditing = false;
	bool m_showBernsteinPoints = false;
	bool m_enableVirtualEdit = false;

	Cursor3D m_cursor;


	std::optional<VirtualPointMapping> m_activeVirtualEdit = std::nullopt;
	std::optional<VirtualPointMapping> PickVirtualPoint(int mouseX, int mouseY, float toleranceSq = 100.0f);
	void BeginVirtualEditAction(int mouseX, int mouseY, const VirtualPointMapping& mapping);
	void ApplyVirtualEditTransform(int mouseX, int mouseY);
#pragma region Menu Methods
	friend class CadGuiManager;
	friend class CamGuiManager;
	CadGuiManager m_cadGuiManager;
	CamGuiManager m_camGuiManager;
	void InitImGui();
	void HandleSurfaceListSelection(Surface* surface, size_t index, bool ctrlHeld, bool shiftHeld);
	void AddTorusAtCursor();
	void AddPointAtCursor(std::shared_ptr<Curve> targetCurve = nullptr);
	std::shared_ptr<Curve> GetSingleSelectedCurve() const;

	template<CurveType T>
	void CreateCurveFromSelectedPoints();
	void AddSurface();

	void DeleteSelectedObjects();
#pragma endregion

	MathLib::Vec3f ScreenToArcballVector(int x, int y, int width, int height);
	std::optional<float3> GetSelectionCenter() const;

#pragma region View/Projection Update Methods
	void UpdateProjectionMatrix();
	void UpdateViewMatrix();
	void SyncPerPassBuffer();
#pragma endregion

	void ClearSelection();
	void ResetSelectionState();
	void HandleObjectSelection(size_t index, bool ctrlHeld, bool shiftHeld);
	void HandleCurveListSelection(Curve* curve, size_t index, bool ctrlHeld, bool shiftHeld);
	std::optional<size_t> PickClosestPoint(int mouseX, int mouseY, float toleranceSq = 100.0f);

	void BeginEditAction(int xPos, int yPos, bool shiftHeld);
	void ApplyEditTransform(int mouseX, int mouseY);

#pragma region stereoscopy
	bool m_enableStereo = false;
	float m_eyeSeparation = 0.2f;
	float m_focalLength = 5.0f;
	MathLib::Vec4f m_leftEyeColor = { 114.0f / 255.0f, 0.0f, 0.0f, 1.0f };
	MathLib::Vec4f m_rightEyeColor = { 0.0f, 58.0f / 255.0f, 58.0f / 255.0f, 1.0f };
#pragma endregion

#pragma region SurfaceCreation

	SurfaceBuilder m_surfaceBuilder;
#pragma endregion

#pragma region Serialization
	std::wstring m_currentFilePath;

	void ActionSave();
	void ActionSaveAs();
	void SaveScene(const std::wstring& filePath);
	void ActionLoad();
	void LoadScene(const std::wstring& filePath);


	template <IsFileDialog DialogType>
	std::wstring ShowFileDialog(
		const std::vector<COMDLG_FILTERSPEC>& filters = {
			{ L"JSON Scene Files (*.json)", L"*.json" },
			{ L"All files (*.*)", L"*.*" }
		},
		const wchar_t* defaultExt = L"json",
		const wchar_t* defaultFileName = L"scene.json");
#pragma endregion

	void ActionMergeSelectedPoints();
	void ActionSealHoles();


	POINT m_boxSelectStart{};
	POINT m_boxSelectCurrent{};
	bool m_isBoxSelecting = false;
	void PerformBoxSelection(bool ctrlHeld, bool shiftHeld);


#pragma region Intersections
	void ActionFindIntersection();
	void RenderTrimTextures(Intersection* intersection) { m_sceneRenderer.RenderTrimTextures(intersection); }
#pragma endregion


	Clock m_clock;
	MillingSimulator m_millingSimulator;
};



template<CurveType T>
inline void CadApplication::CreateCurveFromSelectedPoints()
{
	std::vector<std::weak_ptr<Point>> pts;
	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected && obj->type == ObjectType::Point)
			pts.push_back(std::static_pointer_cast<Point>(obj));
	}

	auto newCurve = std::make_shared<T>(std::move(pts));
	for (const auto& cpWeak : newCurve->m_controlPoints)
	{
		if (auto cp = cpWeak.lock())
			cp->AddDependent(newCurve);
	}
	m_sceneObjects.push_back(newCurve);
}

template<IsFileDialog DialogType>
inline std::wstring CadApplication::ShowFileDialog(
	const std::vector<COMDLG_FILTERSPEC>& filters,
	const wchar_t* defaultExt,
	const wchar_t* defaultFileName)
{
	std::wstring resultPath = L"";
	DialogType* pDialog = nullptr;

	if (FAILED(CoCreateInstance(DialogTraits<DialogType>::Clsid, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDialog))))
		return L"";

	if (!filters.empty())
		pDialog->SetFileTypes(static_cast<UINT>(filters.size()), filters.data());

	if (defaultExt)
		pDialog->SetDefaultExtension(defaultExt);

	if constexpr (std::is_same_v<DialogType, IFileSaveDialog>)
		if (defaultFileName)
			pDialog->SetFileName(defaultFileName);

	if (SUCCEEDED(pDialog->Show(m_window.getHandle())))
	{
		IShellItem* pItem;
		if (SUCCEEDED(pDialog->GetResult(&pItem)))
		{
			PWSTR pszFilePath;
			if (SUCCEEDED(pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath)))
			{
				resultPath = pszFilePath;
				CoTaskMemFree(pszFilePath);
			}
			pItem->Release();
		}
		pDialog->Release();
	}

	return resultPath;
}