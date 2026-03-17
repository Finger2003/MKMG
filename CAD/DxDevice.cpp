#include "pch.h"
#include "DxDevice.h"
#include "DxStructures.h"
#include "Window.h"
#include "exceptions.h"
#include <fstream>

using Microsoft::WRL::ComPtr;
using namespace std;

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
		m_swapChain.GetAddressOf(),
		m_device.GetAddressOf(),
		nullptr,
		m_context.GetAddressOf()
	);
	if (FAILED(hr))
		THROW_DX(hr);
}

ComPtr<ID3D11RenderTargetView> DxDevice::CreateRenderTargetView(const ComPtr<ID3D11Texture2D>& texture) const
{
	ComPtr<ID3D11RenderTargetView> renderTargetView;
	auto hr = m_device->CreateRenderTargetView(texture.Get(), nullptr, renderTargetView.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return renderTargetView;
}

Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> DxDevice::CreateShaderResourceView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const
{
	ComPtr<ID3D11ShaderResourceView> srv;
	auto hr = m_device->CreateShaderResourceView(texture.Get(), nullptr, srv.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return srv;
}

Microsoft::WRL::ComPtr<ID3D11Texture2D> DxDevice::CreateTexture2D(const D3D11_TEXTURE2D_DESC& desc) const
{
	ComPtr<ID3D11Texture2D> texture;
	auto hr = m_device->CreateTexture2D(&desc, nullptr, texture.GetAddressOf());
	if (FAILED(hr))	
		THROW_DX(hr);
	return texture;
}

Microsoft::WRL::ComPtr<ID3D11Buffer> DxDevice::CreateBuffer(const D3D11_BUFFER_DESC& desc, const void* data) const
{
	D3D11_SUBRESOURCE_DATA sdata{};
	sdata.pSysMem = data;

	ComPtr<ID3D11Buffer> buffer;
	auto hr = m_device->CreateBuffer(&desc, data ? &sdata : nullptr, buffer.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return buffer;
}

Microsoft::WRL::ComPtr<ID3D11DepthStencilView> DxDevice::CreateDepthStencilView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const
{
	ComPtr<ID3D11DepthStencilView> dsv;
	auto hr = m_device->CreateDepthStencilView(texture.Get(), nullptr, dsv.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return dsv;
}

Microsoft::WRL::ComPtr<ID3D11DepthStencilView> DxDevice::CreateDepthStencilView(SIZE size) const
{
	auto textureDesc = Texture2DDescription::DepthStencilDescription(size.cx, size.cy);
	ComPtr<ID3D11Texture2D> depthStencilTexture = CreateTexture2D(textureDesc);
	return CreateDepthStencilView(depthStencilTexture);
}

std::vector<BYTE> DxDevice::LoadByteCode(const std::wstring& filename)
{
	ifstream sIn(filename, ios::in | ios::binary);
	if (!sIn)
		THROW(L"Unable to open shader bytecode file: " + filename);

	sIn.seekg(0, ios::end);
	auto bytecodeSize = sIn.tellg();
	sIn.seekg(0, ios::beg);

	vector<BYTE> bytecode(static_cast<unsigned int>(bytecodeSize));

	if (!sIn.read(reinterpret_cast<char*>(bytecode.data()), bytecodeSize))
		THROW(L"Failed to read shader bytecode from file: " + filename);

	sIn.close();
	return bytecode;
}

Microsoft::WRL::ComPtr<ID3D11VertexShader> DxDevice::CreateVertexShader(const std::vector<BYTE>& bytecode) const
{
	ComPtr<ID3D11VertexShader> vertexShader;
	auto hr = m_device->CreateVertexShader(bytecode.data(), bytecode.size(), nullptr, vertexShader.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return vertexShader;
}

Microsoft::WRL::ComPtr<ID3D11PixelShader> DxDevice::CreatePixelShader(const std::vector<BYTE>& bytecode) const
{
	ComPtr<ID3D11PixelShader> pixelShader;
	auto hr = m_device->CreatePixelShader(bytecode.data(), bytecode.size(), nullptr, pixelShader.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return pixelShader;
}

Microsoft::WRL::ComPtr<ID3D11InputLayout> DxDevice::CreateInputLayout(const std::vector<D3D11_INPUT_ELEMENT_DESC>& elements, const std::vector<BYTE>& vsCode) const
{
	ComPtr<ID3D11InputLayout> inputLayout;
	auto hr = m_device->CreateInputLayout(elements.data(), static_cast<UINT>(elements.size()), reinterpret_cast<const void*>(vsCode.data()), vsCode.size(), inputLayout.GetAddressOf());
	if (FAILED(hr))
		THROW_DX(hr);
	return inputLayout;
}

void DxDevice::UpdateBuffer(const Microsoft::WRL::ComPtr<ID3D11Buffer>& buffer, const void* data, size_t count)
{
	D3D11_MAPPED_SUBRESOURCE res;
	auto hr = m_context->Map(buffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &res);
	if (FAILED(hr))
		THROW_DX(hr);
	memcpy(res.pData, data, count);
	m_context->Unmap(buffer.Get(), 0);
}
