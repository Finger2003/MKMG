#pragma once
#include "DxStructures.h"

class Window;

class DxDevice
{
public:
	explicit DxDevice(const Window& window);
	const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& getContext() const { return m_context; }
	const Microsoft::WRL::ComPtr<IDXGISwapChain>& getSwapChain() const { return m_swapChain; }
	ID3D11Device* get() const { return m_device.Get(); }
	ID3D11Device* operator->() const { return m_device.Get(); }

#pragma region Buffer and Texture Creation
	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> CreateRenderTargetView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> CreateShaderResourceView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const;
	Microsoft::WRL::ComPtr<ID3D11Texture2D> CreateTexture2D(const D3D11_TEXTURE2D_DESC& desc) const;
	Microsoft::WRL::ComPtr<ID3D11Buffer> CreateBuffer(const D3D11_BUFFER_DESC& desc, const void* data = nullptr) const;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> CreateDepthStencilView(const Microsoft::WRL::ComPtr<ID3D11Texture2D>& texture) const;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> CreateDepthStencilView(SIZE size) const;
#pragma endregion

#pragma region Shader Creation
	static std::vector<BYTE> LoadByteCode(const std::wstring& filename);
	Microsoft::WRL::ComPtr<ID3D11VertexShader> CreateVertexShader(const std::vector<BYTE>& bytecode) const;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> CreatePixelShader(const std::vector<BYTE>& bytecode) const;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> CreateInputLayout(const std::vector<D3D11_INPUT_ELEMENT_DESC>& elements, const std::vector<BYTE>& vsCode) const;
#pragma endregion


	template<typename T>
	Microsoft::WRL::ComPtr<ID3D11Buffer> CreateVertexBuffer(const std::vector<T>& vertices) const
	{
		D3D11_BUFFER_DESC desc = BufferDescription::VertexBufferDescription(sizeof(T) * vertices.size());
		return CreateBuffer(vertices.data(), desc);
	}
	template<typename T>
	Microsoft::WRL::ComPtr<ID3D11Buffer> CreateIndexBuffer(const std::vector<T>& indices) const
	{
		D3D11_BUFFER_DESC desc = BufferDescription::IndexBufferDescription(sizeof(T) * indices.size());
		return CreateBuffer(indices.data(), desc);
	}
	template<typename T>
	Microsoft::WRL::ComPtr<ID3D11Buffer> CreateConstantBuffer(const std::vector<T>& data) const
	{
		D3D11_BUFFER_DESC desc = BufferDescription::ConstantBufferDescription(sizeof(T) * data.size());
		return CreateBuffer(data.data(), desc);
	}
private:
	Microsoft::WRL::ComPtr<ID3D11Device> m_device;
	Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_context;
	Microsoft::WRL::ComPtr<IDXGISwapChain> m_swapChain;
};

