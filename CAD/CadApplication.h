#pragma once
#include "DxApplication.h"
#include "../MathLib/Mat4f.h"
#include "../MathLib/Vec3f.h"
#include "SceneObject.h"
#include "Torus.h"
#include "Point.h"
#include "Cursor3D.h"
#include "Camera.h"

struct PerPointBuffer
{
	MathLib::Vec4f color;
};


struct PerObjectBuffer
{
	MathLib::Mat4f model;
	MathLib::Vec4f color;
};

struct PerPassBuffer
{
	MathLib::Mat4f viewProj;
	float aspectRatio;
};

enum class InteractionMode
{
	//None,
	//Rotating,
	//Translating,
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
	Scale
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

	virtual ~CadApplication();

	/**
	 * @brief Updates application resources that depend on the size of the window.
	 *
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	void UpdateResources(int width, int height) override;

protected:
	void Render() override; // Renders the scene to the window.

	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> m_depthBuffer;
	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_vertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pixelShader;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> m_layout;

	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_pointVertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pointPixelShader;
	Microsoft::WRL::ComPtr<ID3D11GeometryShader> m_pointGeometryShader;
	//Microsoft::WRL::ComPtr<ID3D11InputLayout> m_pointLayout;


	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerObject;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerPass;

private:
#pragma region Constants
	static constexpr float cMenuWidth = 400.0f;
	static constexpr int cMinWidth = 500;
	static constexpr int cMinHeight = 500;
#pragma endregion
	char m_renameBuffer[128] = {};
	Camera m_camera;

	MathLib::Vec3f m_startArcballVector{};	
	std::vector<std::unique_ptr<SceneObject>> m_sceneObjects;

	InteractionMode m_interactionMode = InteractionMode::None;
	POINT m_lastMousePos{};
	POINT m_startMousePos{};
	SIZE m_renderSize{};

	float3 m_cursorPosition{ 0.0f, 0.0f, 0.0f };

	std::optional<size_t> m_lastClickedIndex = std::nullopt;
	std::optional<size_t> m_nameEditingIndex = std::nullopt;

	MenuState m_menuState = MenuState::List;
	EditAction m_currentEditAction = EditAction::None;

	float m_editAnchorDepth = 0.0f;
	float m_editObjScreenX, m_editObjScreenY;
	float3 m_groupEditCenter;

	mutable std::optional<float3> m_selectionCenterCache = std::nullopt;

	mutable bool m_selectionDirty = true;
	bool m_isEditing = false;

	Cursor3D m_cursor;
#pragma region Menu Methods
	void InitImGui();
	void DrawMenu();
	void DrawListMenu(int selectedCount);
	void DrawEditMenu();
	void DrawTorusMenu(Torus& torus);
	void DrawPointMenu(SceneObject& selectedObj);
	void DrawActionCombo();
	void DrawEditGroupMenu(int selectedCount);
	void DrawCameraSettingsMenu();
	void DrawCursorSettingsMenu();
	void DrawCursor(float3 position, float scale);
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
	void HandleObjectSelection(size_t index, bool ctrlHeld, bool shiftHeld);
	std::optional<size_t> PickClosestPoint(int mouseX, int mouseY, float toleranceSq = 100.0f);
};

