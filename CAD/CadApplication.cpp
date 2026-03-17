#include "pch.h"
#include "CadApplication.h"
#include "../MathLib/Mat4f.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"
#include "Torus.h"

using namespace MathLib;
using namespace std;
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

CadApplication::CadApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	:DxApplication(hInstance, wndWidth, wndHeight, wndTitle)
{
	SIZE wndSize = m_window.getClientSize();
	m_depthBuffer = m_device.CreateDepthStencilView(wndSize);
	auto backBuffer = m_backBuffer.Get();
	m_device.getContext()->OMSetRenderTargets(1, &backBuffer, m_depthBuffer.Get());
	Viewport viewport{ wndSize };
	m_device.getContext()->RSSetViewports(1, &viewport);

	const auto vsByteCode = DxDevice::LoadByteCode(L"VertexShader.cso");
	const auto psByteCode = DxDevice::LoadByteCode(L"PixelShader.cso");
	m_vertexShader = m_device.CreateVertexShader(vsByteCode);
	m_pixelShader = m_device.CreatePixelShader(psByteCode);

	vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	m_layout = m_device.CreateInputLayout(inputElements, vsByteCode);


	// Create the constant buffers for MVP matrices
	m_cbPerPass = m_device.CreateConstantBuffer<PerPassBuffer>();
	m_cbPerObject = m_device.CreateConstantBuffer<PerObjectBuffer>();
	InitImGui();
}

void CadApplication::InitImGui()
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

	ImGui_ImplWin32_Init(m_window.getHandle());
	ImGui_ImplDX11_Init(m_device.get(), m_device.getContext().Get());
}

bool CadApplication::ProcessMessage(WindowMessage& msg)
{
	if (ImGui::GetCurrentContext() == nullptr)
		return WindowApplication::ProcessMessage(msg);

	if (ImGui_ImplWin32_WndProcHandler(m_window.getHandle(), msg.message, msg.wParam, msg.lParam))
		return true;

	ImGuiIO& io = ImGui::GetIO();
	if (io.WantCaptureMouse && 
		(msg.message == WM_LBUTTONDOWN || msg.message == WM_RBUTTONDOWN ||
			msg.message == WM_MOUSEMOVE || msg.message == WM_MOUSEWHEEL))
			return true;

	return DxApplication::ProcessMessage(msg);
}

CadApplication::~CadApplication()
{
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();
}

void CadApplication::UpdateResources(int width, int height)
{
	float aspect = static_cast<float>(width) / height;
	float fovY = 60.0f * (std::numbers::pi_v<float> / 180.0f);
	m_projMatrix = Mat4f::Perspective(fovY, aspect, 0.1f, 100.0f);
	m_viewMatrix = Mat4f::Identity();
	if (m_cbPerPass)
	{
		PerPassBuffer perPassData;
		perPassData.viewProj = m_projMatrix * m_viewMatrix;
		m_device.UpdateBuffer(m_cbPerPass, perPassData);
	}
}

void CadApplication::Render()
{
	SIZE wndSize = m_window.getClientSize();
	int width = wndSize.cx;
	int height = wndSize.cy;
	DrawMenu(width, height);

	m_torus.UpdateMesh(m_device);

	auto& context = m_device.getContext();

	const float clear_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	context->ClearRenderTargetView(m_backBuffer.Get(), clear_color);
	context->ClearDepthStencilView(m_depthBuffer.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);

	context->OMSetRenderTargets(1, m_backBuffer.GetAddressOf(), nullptr);

	context->VSSetConstantBuffers(0, 1, m_cbPerPass.GetAddressOf());

	context->IASetInputLayout(m_layout.Get());
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);


	PerObjectBuffer objData;
	objData.model = Mat4f::Translation(0.0f, -0.1f, -2.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->VSSetConstantBuffers(1, 1, m_cbPerObject.GetAddressOf());

	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, m_torus.GetVertexBuffer().GetAddressOf(), &stride, &offset);
	context->IASetIndexBuffer(m_torus.GetIndexBuffer().Get(), DXGI_FORMAT_R32_UINT, 0);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

	context->DrawIndexed(static_cast<UINT>(m_torus.indices.size()), 0, 0);

	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}

void CadApplication::DrawMenu(int width, int height)
{
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	constexpr float menuWidth = 300.0f;
	ImGui::SetNextWindowPos(ImVec2(width - menuWidth, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(menuWidth, height));

	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove | 
		ImGuiWindowFlags_NoResize | 
		ImGuiWindowFlags_NoCollapse;

	ImGui::Begin("Torus Parameters", nullptr, windowFlags);
	float tempMajor = m_torus.GetMajorRadius();
	float tempMinor = m_torus.GetMinorRadius();
	int tempSegs[2] = { m_torus.GetMajorSegments(), m_torus.GetMinorSegments() };

	ImGui::Text("Torus Settings");
	//if (ImGui::SliderFloat("Major Radius", &tempMajor, m_torus.GetMinorRadius(), Torus::cMaxMajorRadius))
	//	m_torus.SetMajorRadius(tempMajor);
	//if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, m_torus.GetMajorRadius()))
	//	m_torus.SetMinorRadius(tempMinor);
	if (ImGui::SliderFloat("Major Radius", &tempMajor, Torus::cMinMajorRadius, Torus::cMaxMajorRadius))
		m_torus.SetMajorRadius(tempMajor);
	if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, Torus::cMaxMinorRadius))
		m_torus.SetMinorRadius(tempMinor);
	if (ImGui::SliderInt2("Segments (Major, Minor)", tempSegs, Torus::cMinMajorSegments, Torus::cMaxMajorSegments))
		m_torus.SetSegments(tempSegs[0], tempSegs[1]);

	ImGui::End();
	ImGui::Render();
}