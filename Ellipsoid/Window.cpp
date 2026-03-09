#include "pch.h"
#include "window.h"
#include "exceptions.h"

using namespace std;
wstring Window::m_windowClassName = L"Elipsoid Window";


Window::Window(HINSTANCE hInstance, int width, int height, IWindowMessageHandler* messageHandler)
	: Window(hInstance, width, height, m_windowClassName, messageHandler)
{
}

Window::Window(HINSTANCE hInstance, int width, int height, const wstring& title, IWindowMessageHandler* messageHandler)
	: m_hInstance(hInstance), m_messageHandler(messageHandler)
{
	CreateWindowHandle(width, height, title);
}

Window::~Window()
{
	DestroyWindow(m_hWnd);
}

void Window::Show(int nCmdShow)
{
	ShowWindow(m_hWnd, nCmdShow);
}

RECT Window::getClientRectangle() const
{
	RECT r;
	GetClientRect(m_hWnd, &r);
	return r;
}

SIZE Window::getClientSize() const
{
	auto r = getClientRectangle();
	SIZE s = { r.right - r.left, r.bottom - r.top };
	return s;
}

LRESULT Window::WndProc(UINT msg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT paintStruct;

	switch (msg)
	{
	case WM_DESTROY:
		PostQuitMessage(EXIT_SUCCESS);
		break;
	default:
		WindowMessage m = { msg, wParam, lParam, 0 };
		if (m_messageHandler && m_messageHandler->ProcessMessage(m))
		{
			return m.result;
		}
		if (msg == WM_PAINT)
		{
			BeginPaint(m_hWnd, &paintStruct);
			EndPaint(m_hWnd, &paintStruct);
			break;
		}
		return DefWindowProc(m_hWnd, msg, wParam, lParam);
	}
	return 0;
}

bool Window::IsWindowClassRegistered(HINSTANCE hInstance)
{
	WNDCLASSEXW wc;
	if (GetClassInfoExW(hInstance, m_windowClassName.c_str(), &wc) == FALSE)
		return false;

	return wc.lpfnWndProc != static_cast<WNDPROC>(Window::WndProc);
}

void Window::RegisterWindowClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wc{};

	wc.cbSize = sizeof(WNDCLASSEXW);
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
	wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
	//wc.lpszMenuName = nullptr;
	wc.lpszClassName = m_windowClassName.c_str();
	wc.cbWndExtra = sizeof(LONG_PTR);

	if (!RegisterClassExW(&wc))
		THROW_WINAPI;
}

LRESULT CALLBACK Window::WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	Window* wnd;
	if (msg == WM_CREATE)
	{
		auto pcs = reinterpret_cast<LPCREATESTRUCTW>(lParam);
		wnd = static_cast<Window*>(pcs->lpCreateParams);
		SetWindowLongPtrW(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(wnd));
		wnd->m_hWnd = hWnd;
	}
	else
	{
		wnd = reinterpret_cast<Window*>(GetWindowLongPtrW(hWnd, GWLP_USERDATA));
	}

	try
	{
		return wnd ? wnd->WndProc(msg, wParam, lParam) : DefWindowProc(hWnd, msg, wParam, lParam);
	}
	catch (Exception& e)
	{
		MessageBoxW(nullptr, e.getMessage().c_str(), L"Error", MB_OK);
		PostQuitMessage(e.getExitCode());
		return e.getExitCode();
	}
	catch (exception& e)
	{
		string s(e.what());
		MessageBoxW(nullptr, wstring(s.begin(), s.end()).c_str(), L"Error", MB_OK);
	}
	catch (const char* str)
	{
		string s(str);
		MessageBoxW(nullptr, wstring(s.begin(), s.end()).c_str(), L"Error", MB_OK);
	}
	catch (const wchar_t* str)
	{
		MessageBoxW(nullptr, str, L"Error", MB_OK);
	}
	catch (...)
	{
		MessageBoxW(nullptr, L"An unknown error has occurred.", L"Error", MB_OK);
	}
	PostQuitMessage(EXIT_FAILURE);
	return EXIT_FAILURE;
}

void Window::CreateWindowHandle(int width, int height, const std::wstring& windowTitle)
{
	if (!IsWindowClassRegistered(m_hInstance))
		RegisterWindowClass(m_hInstance);

	RECT rect = { 0, 0, width, height };
	//DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
	DWORD style = WS_OVERLAPPEDWINDOW;
	if (!AdjustWindowRect(&rect, style, FALSE))
		THROW_WINAPI;

	m_hWnd = CreateWindowW(m_windowClassName.c_str(), windowTitle.c_str(), style, CW_USEDEFAULT, CW_USEDEFAULT,
		rect.right - rect.left, rect.bottom - rect.top, NULL, NULL, m_hInstance, this);

	if (!m_hWnd)
		THROW_WINAPI;
}