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

Viewport::Viewport(SIZE size)
{
	TopLeftX = 0.0f;
	TopLeftY = 0.0f;
	Width = static_cast<FLOAT>(size.cx);
	Height = static_cast<FLOAT>(size.cy);
	MinDepth = 0.0f;
	MaxDepth = 1.0f;
}

//Texture2DDescription::Texture2DDescription(SIZE size)
//	: Texture2DDescription(size.cx, size.cy)
//{}

Texture2DDescription::Texture2DDescription(UINT width, UINT height)
{
	ZeroMemory(this, sizeof(Texture2DDescription));
	Width = width;
	Height = height;
	MipLevels = 1;
	ArraySize = 1;
	Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	SampleDesc.Count = 1;
	BindFlags = D3D11_BIND_SHADER_RESOURCE;
	//Usage = D3D11_USAGE_DYNAMIC;
	//CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
}

Texture2DDescription Texture2DDescription::DepthStencilDescription(UINT width, UINT height)
{
	Texture2DDescription desc(width, height);
	desc.MipLevels = 1;
	desc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	desc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	return desc;
}

BufferDescription::BufferDescription(UINT bindFlags, size_t byteWidth)
{
	ZeroMemory(this, sizeof(BufferDescription));
	BindFlags = bindFlags;
	ByteWidth = static_cast<UINT>(byteWidth);
	Usage = D3D11_USAGE_DEFAULT;
}

BufferDescription BufferDescription::ConstantBufferDescription(size_t byteWidth)
{
	BufferDescription desc(D3D11_BIND_CONSTANT_BUFFER, byteWidth);
	desc.Usage = D3D11_USAGE_DYNAMIC;
	desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	return desc;
}
