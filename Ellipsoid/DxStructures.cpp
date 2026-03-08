#include "pch.h"
#include "DxStructures.h"

SwapChainDescription::SwapChainDescription(HWND wndHwnd, SIZE wndSize)
{
	ZeroMemory(this, sizeof(SwapChainDescription));
	BufferDesc.Width = wndSize.cx;
	BufferDesc.Height = wndSize.cy;
	BufferDesc.RefreshRate.Denominator = 1;
	BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	SampleDesc.Count = 1;
	BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	BufferCount = 1;
	OutputWindow = wndHwnd;
	Windowed = true;
}

Texture2DDescription::Texture2DDescription(SIZE size)
{
	ZeroMemory(this, sizeof(Texture2DDescription));
	Width = size.cx;
	Height = size.cy;
	MipLevels = 1;
	ArraySize = 1;
	Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	SampleDesc.Count = 1;
	BindFlags = D3D11_BIND_SHADER_RESOURCE;
	Usage = D3D11_USAGE_DYNAMIC;
	CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
}