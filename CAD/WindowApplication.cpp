#include "pch.h"
#include "WindowApplication.h"

WindowApplication::WindowApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	: m_hInstance(hInstance), m_window(hInstance, wndWidth, wndHeight, wndTitle, this)
{}

int WindowApplication::Run(int cmdShow)
{
	m_window.Show(cmdShow);
	return MainLoop();
}

int WindowApplication::MainLoop()
{
	MSG msg{};
	while (GetMessageW(&msg, nullptr, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return static_cast<int>(msg.wParam);
}


