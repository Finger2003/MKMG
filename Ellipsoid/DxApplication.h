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

	DxDevice m_device;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> m_backBuffer;
};

