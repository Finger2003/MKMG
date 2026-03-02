#pragma once

// Wraps a standard WINAPI message.
struct WindowMessage
{
	UINT message;	// The message.
	WPARAM wParam;	// Additional message information.
	LPARAM lParam;	// Additional message information.
	LRESULT result;	// The result of message processing.
};

class IWindowMessageHandler
{
public:
	virtual ~IWindowMessageHandler() = default;

	/**
	* @brief Called when window receives a message.
	* @param [in, out] message contains message ID and its parameters.
	* 
	* If the message is processed, return true and set message.result to the appropriate value.
	* Otherwise, return false to indicate that the message should be passed to the default window procedure.
	*/
	virtual bool ProcessMessage(WindowMessage& message) = 0;
};

/**
* @brief WINAPI window.
* Represents a window that can receive messages and be rendered to.
*/
class Window
{
	static constexpr int m_defaultWindowWidth = 1280;
	static constexpr int m_defaultWindowHeight = 720;

	/**
	* @brief Creates a window.
	* 
	* Creates and empty window of a given width and height with a default title 
	* and registers it with the specified message handler.
	* 
	* Window is not shown until Window::Show(int) is called.
	* @param [in] hInstance Application instance handle.
	* @param [in] width Desired window width.
	* @param [in] height Desired window height.
	* @param [in] messageHandler (optional) Window message handler.
	* @throws WinAPIException Window could not be created.
	* @remark Window class named m_windowClassName is registered it it doesn't exist yet.
	*/
	Window(HINSTANCE hInstance, int width, int height, IWindowMessageHandler* messageHandler = nullptr);

	/**
	 * @brief Creates a window.
	 * 
	 * Creates an empty window with a given width, height and title
	 * and registers it with the specified message handler.
	 * 
	 * @param [in] hInstance Application instance handle.
	 * @param [in] width Desired window width.
	 * @param [in] height Desired window height.
	 * @param [in] title Desired window title.
	 * @param [in] messageHandler (optional) Window message handler.
	 * @throws WinAPIException Window could not be created.
	 * @remark Window class named m_windowClassName is registered it it doesn't exist yet.
	 */
	Window(HINSTANCE hInstance, int width, int height, const std::wstring& title, IWindowMessageHandler* messageHandler = nullptr);

	Window(const Window&) = delete;

	/*
	* @brief Destroys the window
	*/
	virtual ~Window();

	/*
	* @brief Sets windows's show state.
	* @param [in] cmdShow Specifies how the window is to be shown.
	*/
	virtual void Show(int nCmdShow);

	/*
	* @brief Gets size of the windows's client area.
	* @returns SIZE structure containing width and height of the client area.
	*/
	SIZE getClientSize() const;

	/*
	* @bief Gets coordintates of the windows's client area.
	* @returns RECT structure containing top, bottom, left and right coordinates of the client area.
	*/
	RECT getClientRectangle() const;

	/*
	* @brief Gets the WINAPI handle of the window.
	* @returns HWND handle of this window.
	*/
	HWND getHandle() const { return m_hWnd; }

protected:
	/*
	* @brief Processes messsages received by the window.
	* @param [in] message Message ID.
	* @param [in] wParam Additional message information.
	* @param [in] lParam Additional message information.
	*/
	virtual LRESULT WndProc(UINT message, WPARAM wParam, LPARAM lParam);


private:
	/*
	* @brief Checks if the window class with name specified by m_windowClassName is registered 
	* and if its WindowProc callback is set to Window::WndProc.
	* @param [in] hInstance Application instance handle.
	*/
	static bool IsWindowClassRegistered(HINSTANCE hInstance);

	/*
	* @brief Registers the window class for instances of this type if its not yet present.
	* @param [in] hInstance Application instance handle.
	*/
	static void RegisterWindowClass(HINSTANCE hInstance);

	/**
	 * @brief WindowProc callback function for window instances of this type.
	 * 
	 * Callback function called by WINAPI to process messages incoming to windows of this type.
	 * It handles forwarding messages to specific window instances by calling 
	 * non-static member function Window::WndProc on the window instance that is the target 
	 * of the message.
	 * 
	 * @param [in] hWnd Handle of the window receiving the message.
	 * @param [in] message Message ID.
	 * @param [in] wParam Additional message information.
	 * @param [in] lParam Additional message information.
	 * @return Result of message processing.
	 */
	static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);


	static std::wstring m_windowClassName; // Name of the WINAPI window class for windows of this type.

	/**
	 * @brief Creates a window with given width, height and title.
	 * 
	 * @param [in] width Desired window width.
	 * @param [in] height Desired window height.
	 * @param [in] windowTitle Desired window title.
	 * @throws WinAPIException Window could not be created.
	 */
	void CreateWindowHandle(int width, int height, const std::wstring& windowTitle);

	HWND m_hWnd; // Handle of the window.
	HINSTANCE m_hInstance; // Application instance handle.
	IWindowMessageHandler* m_messageHandler; // Window message handler.
};

