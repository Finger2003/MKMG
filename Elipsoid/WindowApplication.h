#pragma once
#include "Window.h"

/**
 * @brief An instancee of WINAPI application.
 * 
 * Handles lifetime of GUI application, including window creation and implementation of the main loop.
 */
class WindowApplication : protected IWindowMessageHandler
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
	explicit WindowApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"Elipsoid");

	/**
	 * @brief Destroys the window and frees application resources.
	 */
	virtual ~WindowApplication() = default;

	/**
	 * @brief Shows the window and runs the main loop of the application.
	 * 
	 * @param cmdShow Specifies how the window is to be shown (passed to ShowWindow).
	 * @return Exit code of the application.
	 */
	int Run(int cmdShow = SW_SHOWNORMAL);

	/**
	 * @brief Gets the handle of the application instance.
	 * 
	 * @return HINSTANCE handle of the application instance.
	 */
	HINSTANCE getHandle() const { return m_hInstance; }


protected:

	/**
	 * @brief Handles system messages received by the window.
	 * 
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed 
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override { return false; }

	/**
	 * @brief Main loop of the application.
	 * 
	 * This funtion is called by Run() to handle program's main loop.
	 * 
	 * @return Application exit code.
	 */
	virtual int MainLoop();

	Window m_window; // The main window of the application.
private:
	HINSTANCE m_hInstance; // Handle of the application instance.
};

