#include "pch.h"
#include "CadApplication.h"

using namespace std;

CadApplication::CadApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	:DxApplication(hInstance, wndWidth, wndHeight, wndTitle)
{
	SIZE wndSize = m_window.getClientSize();
	m_depthBuffer = m_device.CreateDepthStencilView(wndSize);
	auto backBuffer = m_backBuffer.Get();
	m_device.getContext()->OMSetRenderTargets(1, &backBuffer, m_depthBuffer.Get());
	Viewport viewport{ wndSize };
	m_device.getContext()->RSSetViewports(1, &viewport);

	//const auto vsByteCode = DxDevice::LoadByteCode(L"VertexShader.cso");
	//const auto psByteCode = DxDevice::LoadByteCode(L"PixelShader.cso");
	//m_vertexShader = m_device.CreateVertexShader(vsByteCode);
	//m_pixelShader = m_device.CreatePixelShader(psByteCode);

	//vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
	//	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	//};
	//m_layout = m_device.CreateInputLayout(inputElements, vsByteCode);
}
