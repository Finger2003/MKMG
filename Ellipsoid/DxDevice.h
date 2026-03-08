#pragma once
class Window;

class DxDevice
{
public:
	explicit DxDevice(const Window& window);
	const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& getContext() const { return m_context; }
	const Microsoft::WRL::ComPtr<IDXGISwapChain>& getSwapChain() const { return m_swapChain; }
	ID3D11Device* get() const { return m_device.Get(); }
	ID3D11Device* operator->() const { return m_device.Get(); }

	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> CreateRenderTargetView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> CreateShaderResourceView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const;

	Microsoft::WRL::ComPtr<ID3D11Texture2D> CreateTexture2D(const D3D11_TEXTURE2D_DESC& desc) const;

private:
	Microsoft::WRL::ComPtr<ID3D11Device> m_device;
	Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_context;
	Microsoft::WRL::ComPtr<IDXGISwapChain> m_swapChain;
};

