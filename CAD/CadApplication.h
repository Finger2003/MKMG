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

//struct 

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
	Edit
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


	MathLib::Mat4f m_viewMatrix;
	MathLib::Mat4f m_projMatrix;
	MathLib::Mat4f m_projViewMatrix;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerObject;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerPass;

private:
	InteractionMode m_interactionMode = InteractionMode::None;
	POINT m_lastMousePos{};
	POINT m_startMousePos{};
	MathLib::Vec3f m_startArcballVector{};
	//Torus m_torus;
	

	std::vector<std::unique_ptr<SceneObject>> m_sceneObjects;

	//std::vector<std::unique_ptr<Torus>> m_toruses;
	//std::vector<VertexPosition> m_points;
	//Microsoft::WRL::ComPtr<ID3D11Buffer> m_pointsBuffer;
	bool m_pointsDirty = false;
	void UpdatePointsBuffer();

	void DrawMenu();
	void InitImGui();
	MathLib::Vec3f ScreenToArcballVector(int x, int y, int width, int height);

	float m_fovY = 60.0f;
	float m_nearPlane = 0.1f;
	float m_farPlane = 100.0f;
	float m_panScaleFactor = 0.0f;
	void UpdateProjectionMatrix();

	static constexpr float cMenuWidth = 400.0f;
	SIZE m_renderSize{};

	static constexpr int cMinWidth = 500;
	static constexpr int cMinHeight = 500;

	Cursor3D m_cursor;
	float3 m_cursorPosition{ 0.0f, 0.0f, 0.0f };

	void DrawCursor(float3 position, float scale);

	Camera m_camera;

	int m_lastClickedIndex = -1;

	void DeleteSelectedObjects();

	int m_nameEditingIndex = -1;
	char m_renameBuffer[128] = {};
	std::optional<float3> GetSelectionCenter() const;

	MenuState m_menuState = MenuState::List;
	EditAction m_currentEditAction = EditAction::None;
};

