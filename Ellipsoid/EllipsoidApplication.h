#pragma once
#include "DxApplication.h"
#include "../MathLib/Vec3d.h"
#include "../MathLib/Mat4d.h"
#include "dxDevice.h"

//struct Elipsoid
//{
//	double rx{ 1.0 }, ry{ 1.0 }, rz{ 1.0 }; // Radii along x, y and z axes.
//	double posx{}, posy{}, posz{}; // Position of the center of the elipsoid.
//	double rotX{ 0.0 }, rotY{ 0.0 }, rotZ{ 0.0 }; // Rotation angles around x, y and z axes.
//	double scale = 1.0;
//
//	static constexpr MathLib::Vec3d color = MathLib::Vec3d(1.0, 1.0, 0.0);
//};

struct ViewPort
{
	double width;
	double height;
	double aspectRatio;
};

struct Ellipsoid
{
	//MathLib::Vec3d radii{ 1.5, 1.0, 1.25 }; // Radii along x, y and z axes.
	MathLib::Vec3d radii{ 1.5, 1.0, 1.0 }; // Radii along x, y and z axes.
	MathLib::Vec3d position{}; // Position of the center of the elipsoid.
	//MathLib::Vec3d rotation{}; // Rotation angles around x, y and z axes.

	MathLib::Mat4d rotationMatrix = MathLib::Mat4d::Identity();
	//MathLib::Vec3d scale{ 1.0, 1.0, 1.0 };
	MathLib::Vec3d scale{ 0.7, 0.7, 0.7 };

	static MathLib::Vec3d color; // Elipsoid color.

	MathLib::Mat4d DMprim;
	bool needsUpdate = true;

	void UpdateDMprimMatrix();
};

enum class InteractionMode 
{
	None,
	Rotating,
	Translating,
};

class EllipsoidApplication : public DxApplication
{
public:
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
	explicit EllipsoidApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"Elipsoid");

	/**
	 * @brief Destroys the window and frees application resources.
	 */
	virtual ~EllipsoidApplication();

protected:

	/**
	 * @brief Handles system messages received by the window.
	 *
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override;

	/**
	 * @brief Main loop of the application.
	 *
	 * This funtion is called by Run() to handle program's main loop.
	 *
	 * @return Application exit code.
	 */
	//int MainLoop() override;

	void Render() override; // Renders the elipsoid to the window.

	/**
	 * @brief Updates application resources that depend on the size of the window.
	 *
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	void UpdateResources(int width, int height) override;
private:
	static constexpr LONG menuWidth = 400;
	static constexpr int cMaxStepExponent = 6;
	static constexpr int cInitialStepExponent = 3;
	int m_initialStep = 8;
	int m_currentStep = m_initialStep;
	POINT m_lastMousePos{};
	Ellipsoid m_ellipsoid;
	InteractionMode m_interactionMode = InteractionMode::None;
	int m_specularExponent = 2;

	std::vector<uint32_t> m_pixelData;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> m_cpuTexture;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_cpuTextureView;

	float m_backgroundColor[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

	static constexpr int m_minWidth = 500;
	static constexpr int m_minHeight = 500;

	void DrawEllipsoid();
	void DrawMenu(int drawWidth, int height);
	void UpdateViewport(int width, int height);

	ViewPort m_viewport{};
};

