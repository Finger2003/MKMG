#include "pch.h"
#include "DxDevice.h"
#include "DxStructures.h"
#include "Window.h"
#include "exceptions.h"

using Microsoft::WRL::ComPtr;

DxDevice::DxDevice(const Window& window)
{
	SwapChainDescription desc{ window.getHandle(), window.getClientSize()};
	//ID3D11Device* device = nullptr;
	//ID3D11DeviceContext* context = nullptr;
	//IDXGISwapChain* swapChain = nullptr;

	auto hr = D3D11CreateDeviceAndSwapChain(
		nullptr,
		D3D_DRIVER_TYPE_HARDWARE,
		nullptr,
		0,
		nullptr,
		0,
		D3D11_SDK_VERSION,
		&desc,
		m_swapChain.ReleaseAndGetAddressOf(),
		m_device.ReleaseAndGetAddressOf(),
		nullptr,
		m_context.ReleaseAndGetAddressOf()
	);
	if (FAILED(hr))
		THROW_WINAPI;
}

ComPtr<ID3D11RenderTargetView> DxDevice::CreateRenderTargetView(const ComPtr<ID3D11Texture2D>& texture) const
{
	ComPtr<ID3D11RenderTargetView> renderTargetView;
	auto hr = m_device->CreateRenderTargetView(texture.Get(), nullptr, renderTargetView.GetAddressOf());
	if (FAILED(hr))
		THROW_WINAPI;
	return renderTargetView;
}

Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> DxDevice::CreateShaderResourceView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const
{
	ComPtr<ID3D11ShaderResourceView> srv;
	auto hr = m_device->CreateShaderResourceView(texture.Get(), nullptr, srv.GetAddressOf());
	if (FAILED(hr))
		THROW_WINAPI;
	return srv;
}

Microsoft::WRL::ComPtr<ID3D11Texture2D> DxDevice::CreateTexture2D(const D3D11_TEXTURE2D_DESC& desc) const
{
	ComPtr<ID3D11Texture2D> texture;
	auto hr = m_device->CreateTexture2D(&desc, nullptr, texture.GetAddressOf());
	if (FAILED(hr))	
		THROW_WINAPI;
	return texture;
}
