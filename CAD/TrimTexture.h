#pragma once
#include "DxDevice.h"

struct TrimTexture
{
	Microsoft::WRL::ComPtr<ID3D11Texture2D> texture;
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> rtv;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> srv;
	int width;
	int height;

	void Init(const DxDevice& device, int w = 512, int h = 512)
	{
		width = w;
		height = h;

		D3D11_TEXTURE2D_DESC texDesc = {};
		texDesc.Width = width;
		texDesc.Height = height;
		texDesc.MipLevels = 1;
		texDesc.ArraySize = 1;
		texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		texDesc.SampleDesc.Count = 1;
		texDesc.Usage = D3D11_USAGE_DEFAULT;
		texDesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;

		device.get()->CreateTexture2D(&texDesc, nullptr, texture.GetAddressOf());
		device.get()->CreateRenderTargetView(texture.Get(), nullptr, rtv.GetAddressOf());
		device.get()->CreateShaderResourceView(texture.Get(), nullptr, srv.GetAddressOf());
	}
};