#include "pch.h"
#include "CadApplication.h"
#include "../MathLib/Mat4f.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"

using namespace MathLib;
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

	// ImGui
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

CadApplication::~CadApplication()
{
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();
}

void Torus::GenerateMesh()
{
	GenerateVertices();
	GenerateIndices();
}

void Torus::GenerateVertices()
{
	vertices.clear();
	vertices.reserve(static_cast<size_t>(majorSegments) * minorSegments);
	for (int i = 0; i < majorSegments; i++)
	{
		float beta = i * 2.0f * std::numbers::pi_v<float> / majorSegments;
		float cosBeta = std::cos(beta);
		float sinBeta = std::sin(beta);
		for (int j = 0; j < minorSegments; j++)
		{
			float alpha = j * 2.0f * std::numbers::pi_v<float> / minorSegments;
			float cosAlpha = std::cos(alpha);
			float sinAlpha = std::sin(alpha);
			vertices.emplace_back(
				(majorRadius + minorRadius * cosAlpha) * cosBeta,
				minorRadius * sinAlpha,
				-(majorRadius + minorRadius * cosAlpha) * sinBeta
			);
		}
	}
}

void Torus::GenerateIndices()
{
	indices.clear();
	vertices.reserve(static_cast<size_t>(majorSegments) * minorSegments * 2 * 2); // 2 edges per vertex, 2 vertices per edge
	for (int i = 0; i < majorSegments; i++)
	{
		for (int j = 0; j < minorSegments; j++)
		{
			int current = i * minorSegments + j;

			// Minor Circle Edges
			// (i, j) -> (i, j + 1)
			int nextMinor = i * minorSegments + (j + 1) % minorSegments;
			indices.push_back(current);
			indices.push_back(nextMinor);

			// Major Circle Edges
			// (i, j) -> (i + 1, j)
			int nextMajor = ((i + 1) % majorSegments) * minorSegments + j;
			indices.push_back(current);
			indices.push_back(nextMajor);
		}
	}
}
