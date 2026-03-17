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

static pair<float, float> CalculateCoordsFromPixel(float x, float y, float width, float height)
{
	float normX = (x / width) * 2.0f - 1.0f;
	float normY = 1.0f - (y / height) * 2.0f; // Invert Y coordinate
	return { normX, normY };
}

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

MathLib::Vec3f CadApplication::ScreenToArcballVector(int x, int y, int width, int height)
{
	auto [normX, normY] = CalculateCoordsFromPixel(x, y, width, height);
	Vec3f p(normX, normY, 0.0f);
	float length_sqr = p.length_sqr();

	if (length_sqr <= 1.0f)
		p.z = std::sqrt(1.0f - length_sqr);
	else
		p = p.normalize();

	return p;
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

	int xPos = (int)(short)LOWORD(msg.lParam);
	int yPos = (int)(short)HIWORD(msg.lParam);
	SIZE wndSize = m_window.getClientSize();

	switch (msg.message)
	{
	case WM_LBUTTONDOWN:
		m_interactionMode = InteractionMode::Rotating;
		m_lastMousePos = { xPos, yPos };
		m_startMousePos = { xPos, yPos };
		m_startArcballVector = ScreenToArcballVector(xPos, yPos, wndSize.cx, wndSize.cy);
		m_torus.m_baseRotationMatrix = m_torus.m_rotationMatrix;
		SetCapture(m_window.getHandle());
		return true;
	case WM_RBUTTONDOWN:
		m_interactionMode = InteractionMode::Translating;
		m_lastMousePos = { xPos, yPos };
		SetCapture(m_window.getHandle());
		return true;
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
		m_interactionMode = InteractionMode::None;
		ReleaseCapture();
		return true;
	case WM_MOUSEMOVE:
		if (m_interactionMode != InteractionMode::None)
		{
			int dx = xPos - m_lastMousePos.x;
			int dy = yPos - m_lastMousePos.y;
			if (m_interactionMode == InteractionMode::Rotating)
			{
				Vec3f currentArcballVector = ScreenToArcballVector(xPos, yPos, wndSize.cx, wndSize.cy);
				float dot = std::clamp(Vec3f::dot(m_startArcballVector, currentArcballVector), -1.0f, 1.0f);
				float angle = std::acos(dot) * 2.0f;
				Vec3f rotationAxis = Vec3f::cross(m_startArcballVector, currentArcballVector);

				if (rotationAxis.length_sqr() > 1e-6f)
				{
					Mat4f rot = Mat4f::RotationAxis(rotationAxis.normalize(), angle);
					m_torus.m_rotationMatrix = rot * m_torus.m_baseRotationMatrix;
					Vec3f euler = Mat4f::ExtractEulerAngles(m_torus.m_rotationMatrix);
					m_torus.m_eulerAngles = { euler.x, euler.y, euler.z };
				}

				//float sensitivity = 0.01f;
				//m_torus.m_eulerAngles.x += dy * sensitivity;
				//m_torus.m_eulerAngles.y += dx * sensitivity;

				//// Rebuild the matrix exactly as we do in the UI
				//Mat4f rotX = Mat4f::RotationX(m_torus.m_eulerAngles.x);
				//Mat4f rotY = Mat4f::RotationY(m_torus.m_eulerAngles.y);
				//Mat4f rotZ = Mat4f::RotationZ(m_torus.m_eulerAngles.z);

				//m_torus.m_rotationMatrix = rotY * rotX * rotZ;

	

				//int totalDx = xPos - m_startMousePos.x;
				//int totalDy = yPos - m_startMousePos.y;
				//float sensitivity = 0.01f;
				//float angleX = totalDy * sensitivity;
				//float angleY = totalDx * sensitivity;

				//Mat4f rotX = Mat4f::RotationX(angleX);
				//Mat4f rotY = Mat4f::RotationY(angleY);

				//m_torus.m_rotationMatrix = rotY * rotX * m_torus.m_baseRotationMatrix;
				
			}
			else if (m_interactionMode == InteractionMode::Translating)
			{
				SIZE wndSize = m_window.getClientSize();
				float width = static_cast<float>(wndSize.cx);
				float height = static_cast<float>(wndSize.cy);

				float fovY = 60.0f * (std::numbers::pi_v<float> / 180.0f);
				float distanceZ = std::abs(m_torus.m_position.z);

				float frustumHeight = 2.0f * distanceZ * std::tan(fovY / 2.0f);
				float worldUnitsPerPixelY = frustumHeight / height;

				float aspect = width / height;
				float frustumWidth = frustumHeight * aspect;
				float worldUnitsPerPixelX = frustumWidth / width;

				m_torus.m_position.x += dx * worldUnitsPerPixelX;
				m_torus.m_position.y -= dy * worldUnitsPerPixelY;

				//m_torus.m_modelMatrix = Mat4f::Translation(m_torus.m_position.x, m_torus.m_position.y, m_torus.m_position.z);
			}
			Mat4f translation = Mat4f::Translation(m_torus.m_position.x, m_torus.m_position.y, m_torus.m_position.z);
			Mat4f scaling = Mat4f::Scaling(m_torus.m_scale);
			m_torus.m_modelMatrix = translation * m_torus.m_rotationMatrix * scaling;
			m_lastMousePos = { xPos, yPos };
		}
		return true;
	case WM_MOUSEWHEEL:
	{
		short zDelta = (short)HIWORD(msg.wParam);
		WORD fwKeys = LOWORD(msg.wParam);

		if (fwKeys & MK_CONTROL)
		{
			m_torus.m_position.z += (zDelta > 0) ? 0.1 : -0.1; // Move along z-axis.
		}
		//else if (fwKeys & MK_SHIFT)
		//{
		//	// Rotate around z-axis.
		//	//m_ellipsoid.rotation.z += (zDelta > 0) ? 0.1 : -0.1;
		//	Mat4d rotZ = Mat4d::RotationZ((zDelta > 0) ? 0.1 : -0.1);
		//	m_ellipsoid.rotationMatrix = rotZ * m_ellipsoid.rotationMatrix;
		//	m_ellipsoid.rotationMatrix.Orthonormalize3x3(); // Keep the rotation matrix orthonormal to prevent distortion.
		//}
		else
		{
			float scaleFactor = (zDelta > 0) ? 1.1f : 0.9f;
			m_torus.m_scale *= scaleFactor;
		}
		Mat4f translation = Mat4f::Translation(m_torus.m_position.x, m_torus.m_position.y, m_torus.m_position.z);
		Mat4f scaling = Mat4f::Scaling(m_torus.m_scale);
		m_torus.m_modelMatrix = translation * m_torus.m_rotationMatrix * scaling;
		return true;
	}
	}


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
	m_depthBuffer = m_device.CreateDepthStencilView(SIZE{ width, height });
	Viewport viewport{ SIZE{ width, height } };
	m_device.getContext()->RSSetViewports(1, &viewport);

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
	objData.model = m_torus.m_modelMatrix;
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
	if (ImGui::SliderFloat("Major Radius", &tempMajor, Torus::cMinMajorRadius, Torus::cMaxMajorRadius))
		m_torus.SetMajorRadius(tempMajor);
	if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, Torus::cMaxMinorRadius))
		m_torus.SetMinorRadius(tempMinor);
	if (ImGui::SliderInt2("Segments (Major, Minor)", tempSegs, Torus::cMinMajorSegments, Torus::cMaxMajorSegments))
		m_torus.SetSegments(tempSegs[0], tempSegs[1]);

	ImGui::Separator();
	ImGui::Text("Transformations");
	bool transformChanged = false;

	if (ImGui::DragFloat3("Position", &m_torus.m_position.x, 0.01f))
		transformChanged = true;
	if (ImGui::DragFloat3("Rotation (Euler angles)", &m_torus.m_eulerAngles.x, 0.01f))
	{
		Mat4f rotX = Mat4f::RotationX(m_torus.m_eulerAngles.x);
		Mat4f rotY = Mat4f::RotationY(m_torus.m_eulerAngles.y);
		Mat4f rotZ = Mat4f::RotationZ(m_torus.m_eulerAngles.z);
		m_torus.m_rotationMatrix = rotY * rotX * rotZ;
		transformChanged = true;
	}
	if (ImGui::Button("Reset Rotation"))
	{
		m_torus.m_eulerAngles = { 0, 0, 0 };
		m_torus.m_rotationMatrix = Mat4f::Identity();
		m_torus.m_baseRotationMatrix = Mat4f::Identity();
		transformChanged = true;
	}

	if (ImGui::DragFloat("Scale", &m_torus.m_scale, 0.01f, 0.1f, 100.0f))
		transformChanged = true;

	if (transformChanged)
	{
		Mat4f translation = Mat4f::Translation(m_torus.m_position.x, m_torus.m_position.y, m_torus.m_position.z);
		Mat4f scaling = Mat4f::Scaling(m_torus.m_scale);
		m_torus.m_modelMatrix = translation * m_torus.m_rotationMatrix * scaling;
	}

	ImGui::End();
	ImGui::Render();
}