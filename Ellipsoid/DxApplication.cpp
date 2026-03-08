#include "pch.h"
#include "DxApplication.h"

using Microsoft::WRL::ComPtr;

DxApplication::DxApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	: WindowApplication(hInstance, wndWidth, wndHeight, wndTitle), m_device(m_window)
{
	ComPtr<ID3D11Texture2D> backTexture;
	m_device.getSwapChain()->GetBuffer(0, IID_PPV_ARGS(backTexture.GetAddressOf()));
	m_backBuffer = m_device.CreateRenderTargetView(backTexture);
}

int DxApplication::MainLoop()
{
	MSG msg{};
	do
	{
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			Render();
			m_device.getSwapChain()->Present(0, 0);
		}
	} while (msg.message != WM_QUIT);
	return msg.wParam;
}
