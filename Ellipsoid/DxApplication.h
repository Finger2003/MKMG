#pragma once
#include "WindowApplication.h"
#include "DxDevice.h"

class DxApplication : public WindowApplication
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
	explicit DxApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"DxApp");

	virtual ~DxApplication() = default;

protected:
	/**
	 * @brief Main loop of the application.
	 *
	 * This funtion is called by Run() to handle program's main loop.
	 *
	 * @return Application exit code.
	 */
	int MainLoop() override;

	virtual void Render() = 0; // Renders the scene to the window.


	/**
	 * @brief Handles system messages received by the window.
	 *
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override;

	/**
	 * @brief Called when the window is resized.
	 * 
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	void OnResize(int width, int height);

	/**
	 * @brief Updates application resources that depend on the size of the window.
	 * 
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	virtual void UpdateResources(int width, int height) = 0;

	DxDevice m_device;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> m_backBuffer;
};

