#include "pch.h"
#include "CadApplication.h"
#include "../MathLib/Mat4f.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"


using namespace MathLib;
using namespace std;
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static MathLib::Vec3f ScreenToObjectArcballVector(int mouseX, int mouseY, float objScreenX, float objScreenY, float radius)
{
	float dx = (mouseX - objScreenX) / radius;
	float dy = (objScreenY - mouseY) / radius; // Invert Y because screen Y goes down

	MathLib::Vec3f p(dx, dy, 0.0f);
	float length_sqr = p.length_sqr();

	if (length_sqr <= 0.5f)
	{
		p.z = std::sqrt(1.0f - length_sqr);
	}
	else
	{
		p.z = 0.5f / std::sqrt(length_sqr);
	}

	//if (length_sqr <= 1.0f)
	//	p.z = std::sqrt(1.0f - length_sqr); // Inside the sphere
	//else
	//	p = p.normalize(); // Outside the sphere (maps to the edge)

	return p.normalize();
}

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
	const auto pointVsByteCode = DxDevice::LoadByteCode(L"PointVS.cso");
	const auto pointPsByteCode = DxDevice::LoadByteCode(L"PointPS.cso");
	const auto pointGsByteCode = DxDevice::LoadByteCode(L"PointGS.cso");
	m_vertexShader = m_device.CreateVertexShader(vsByteCode);
	m_pixelShader = m_device.CreatePixelShader(psByteCode);
	m_pointVertexShader = m_device.CreateVertexShader(pointVsByteCode);
	m_pointPixelShader = m_device.CreatePixelShader(pointPsByteCode);
	m_pointGeometryShader = m_device.CreateGeometryShader(pointGsByteCode);

	vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	m_layout = m_device.CreateInputLayout(inputElements, vsByteCode);


	// Create the constant buffers for MVP matrices
	m_cbPerPass = m_device.CreateConstantBuffer<PerPassBuffer>();
	m_cbPerObject = m_device.CreateConstantBuffer<PerObjectBuffer>();
	InitImGui();


	Cursor3D::InitSharedGeometry(m_device);
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

void CadApplication::UpdateProjectionMatrix()
{
	int width = m_renderSize.cx;
	int height = m_renderSize.cy;
	float aspect = static_cast<float>(width) / height;
	float fovY = m_fovY * (std::numbers::pi_v<float> / 180.0f);
	m_panScaleFactor = 2.0f * std::tan(fovY / 2.0f) / height;

	m_nearPlane = std::max(m_nearPlane, 0.01f); // Ensure near plane is positive and not too close to zero.
	m_farPlane = std::max(m_farPlane, m_nearPlane + 0.01f); // Ensure far plane is greater than near plane.
	m_projMatrix = Mat4f::Perspective(fovY, aspect, m_nearPlane, m_farPlane);

	m_camera.UpdateMatrices();
	m_viewMatrix = m_camera.GetViewMatrix();
	m_projViewMatrix = m_projMatrix * m_viewMatrix;
	if (m_cbPerPass)
	{
		PerPassBuffer perPassData;
		perPassData.viewProj = m_projViewMatrix;
		perPassData.aspectRatio = aspect;
		m_device.UpdateBuffer(m_cbPerPass, perPassData);
	}
}

void CadApplication::DrawCursor(float3 position, float scale)
{
	auto& context = m_device.getContext();

	//context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

	PerObjectBuffer objData;
	Mat4f translation = Mat4f::Translation(position.x, position.y, position.z);
	Mat4f scaling = Mat4f::Scaling(scale);

	// X axis - Red
	objData.model = translation * scaling;
	objData.color = { 1.0f, 0.0f, 0.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);

	// Y axis - Green (Rotate X arrow 90 degrees around Z axis)
	objData.model = translation * Mat4f::RotationZ(std::numbers::pi_v<float> / 2.0f) * scaling;
	objData.color = { 0.0f, 1.0f, 0.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);

	// Z axis - Blue (Rotate X arrow -90 degrees around Y axis)
	objData.model = translation * Mat4f::RotationY(-std::numbers::pi_v<float> / 2.0f) * scaling;
	objData.color = { 0.0f, 0.0f, 1.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);
}

void CadApplication::DeleteSelectedObjects()
{
	erase_if(m_sceneObjects, [](const auto& obj) { return obj->selected; });
	m_lastClickedIndex = -1;
}

std::optional<float3> CadApplication::GetSelectionCenter() const
{
	Vec4f sum; // .w counts the number of selected objects. Max possible count is 16 777 216 due to float precision.
	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected)
			sum += Vec4f(obj->m_position.x, obj->m_position.y, obj->m_position.z, 1.0f);
	}

	if (sum.w < 1.0f) // No objects selected
		return std::nullopt;

	Vec4f center = sum / sum.w;
	return float3( center.x, center.y, center.z );
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
	//SIZE wndSize = m_window.getClientSize();

	switch (msg.message)
	{
	case WM_GETMINMAXINFO:
	{
		MINMAXINFO* mmi = reinterpret_cast<MINMAXINFO*>(msg.lParam);
		mmi->ptMinTrackSize.x = cMinWidth;
		mmi->ptMinTrackSize.y = cMinHeight;
		msg.result = 0;
		return true;
	}
	case WM_LBUTTONDOWN:
	{
		if (m_menuState == MenuState::Edit && m_currentEditAction != EditAction::None)
		{
			m_isEditing = true;
			m_lastMousePos = { xPos, yPos };
			auto& obj = m_sceneObjects[m_lastClickedIndex];

			Vec4f worldPos = Vec4f(obj->m_position.x, obj->m_position.y, obj->m_position.z, 1.0f);
			Vec4f clipPos = m_projViewMatrix * worldPos;
			clipPos /= clipPos.w;

			m_editObjScreenX = (clipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
			m_editObjScreenY = (1.0f - clipPos.y) * 0.5f * m_renderSize.cy;
			m_startArcballVector = ScreenToObjectArcballVector(xPos, yPos, m_editObjScreenX, m_editObjScreenY, 150.0f);

			Vec3f objPos = Vec3f(obj->m_position.x, obj->m_position.y, obj->m_position.z);
			Vec3f toObj = objPos - m_camera.GetPosition();
			m_editAnchorDepth = Vec3f::dot(toObj, m_camera.GetForwardVector());
			if (obj->type == ObjectType::Torus)
			{
				auto torus = static_cast<Torus*>(obj.get());
				torus->m_baseRotationMatrix = torus->m_rotationMatrix;
			}
			SetCapture(m_window.getHandle());
			return true;
		}

		SceneObject* closestObj = nullptr;
		size_t closestIndex = -1;
		float minZ = std::numeric_limits<float>::max();
		float toleranceSq = 100.0f;

		for (size_t i = 0; i < m_sceneObjects.size(); i++)
		{
			auto& obj = m_sceneObjects[i];
			if (obj->type != ObjectType::Point)
				continue;

			Vec4f worldPos = Vec4f(obj->m_position.x, obj->m_position.y, obj->m_position.z, 1.0f);
			Vec4f clipPos = m_projViewMatrix * worldPos; 
			if (clipPos.w <= 0)
				continue;

			clipPos /= clipPos.w;
			float screenX = (clipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
			float screenY = (1.0f - clipPos.y) * 0.5f * m_renderSize.cy;
			float distSq = (screenX - xPos) * (screenX - xPos) + (screenY - yPos) * (screenY - yPos);
			if (distSq < toleranceSq && clipPos.z < minZ)
			{
				minZ = clipPos.z;
				closestIndex = i;
				closestObj = obj.get();
			}
		}

		WORD fwKeys = LOWORD(msg.wParam);
		if (closestObj)
		{
			if (fwKeys & MK_CONTROL)
			{
				closestObj->selected = !closestObj->selected;
				m_lastClickedIndex = closestIndex;
			}
			else 
			{
				for (auto& o : m_sceneObjects)
					o->selected = false;
				closestObj->selected = true;
				m_lastClickedIndex = closestIndex;
			}
		}
		else
		{
			if (!(fwKeys & MK_CONTROL))
			{
				for (auto& o : m_sceneObjects)
					o->selected = false;
				m_lastClickedIndex = -1;
			}

			auto [normX, normY] = CalculateCoordsFromPixel(xPos, yPos, m_renderSize.cx, m_renderSize.cy);

			float distance = m_camera.GetDistance();

			float fovY_rad = m_fovY * (std::numbers::pi_v<float> / 180.0f);
			float aspect = static_cast<float>(m_renderSize.cx) / m_renderSize.cy;

			float planeHeight = 2.0f * distance * std::tan(fovY_rad / 2.0f);
			float planeWidth = planeHeight * aspect;

			float localX = normX * (planeWidth / 2.0f);
			float localY = normY * (planeHeight / 2.0f);
			float localZ = -distance;

			Vec4f localPos(localX, localY, localZ, 1.0f);
			Vec4f worldPos = m_camera.GetInverseViewMatrix() * localPos;

			m_cursorPosition = { worldPos.x, worldPos.y, worldPos.z };
		}
	}
	return true;
	case WM_MBUTTONDOWN:
		m_interactionMode = InteractionMode::Orbiting;
		m_lastMousePos = { xPos, yPos };
		SetCapture(m_window.getHandle());
		return true;
	case WM_RBUTTONDOWN:
		m_interactionMode = InteractionMode::Panning;
		m_lastMousePos = { xPos, yPos };
		SetCapture(m_window.getHandle());
		return true;
		//m_interactionMode = InteractionMode::Translating;
		//m_lastMousePos = { xPos, yPos };
		//SetCapture(m_window.getHandle());
		//return true;
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
	case WM_MBUTTONUP:
		m_isEditing = false;
		m_interactionMode = InteractionMode::None;
		ReleaseCapture();
		return true;
	case WM_MOUSEMOVE:
	{
		int dx = xPos - m_lastMousePos.x;
		int dy = yPos - m_lastMousePos.y;

		if (m_isEditing)
		{
			auto& obj = m_sceneObjects[m_lastClickedIndex];
			if (m_currentEditAction >= EditAction::TranslateFree && m_currentEditAction <= EditAction::TranslateZ)
			{
				float moveX = dx * 0.01f;
				//float moveY = -dy * 0.01f;

				if (m_currentEditAction == EditAction::TranslateFree)
				{
					Vec3f objPos = Vec3f(obj->m_position.x, obj->m_position.y, obj->m_position.z);
					Vec3f toObj = objPos - m_camera.GetPosition();
					//float depth = Vec3f::dot(toObj, m_camera.GetForwardVector());
					float depth = m_editAnchorDepth;
					float unitsPerPixel = m_panScaleFactor * std::abs(depth);

					Vec3f worldDelta = (m_camera.GetRightVector() * dx - m_camera.GetUpVector() * dy) * unitsPerPixel;
					Vec3f newPos = objPos + worldDelta;
					obj->m_position.x = newPos.x;
					obj->m_position.y = newPos.y;
					obj->m_position.z = newPos.z;
				}
				else if (m_currentEditAction == EditAction::TranslateX)
					obj->m_position.x += moveX;
				else if (m_currentEditAction == EditAction::TranslateY)
					obj->m_position.y += moveX;
				else if (m_currentEditAction == EditAction::TranslateZ)
					obj->m_position.z += moveX;
			}
			else if (obj->type == ObjectType::Torus)
			{
				auto torus = static_cast<Torus*>(obj.get());

				if (m_currentEditAction == EditAction::RotateFree)
				{
					//Vec3f currentArcballVector = ScreenToArcballVector(xPos, yPos, m_renderSize.cx, m_renderSize.cy);
					Vec3f currentArcballVector = ScreenToObjectArcballVector(xPos, yPos, m_editObjScreenX, m_editObjScreenY, 150.0f);
					float dot = std::clamp(Vec3f::dot(m_startArcballVector, currentArcballVector), -1.0f, 1.0f);
					float angle = std::acos(dot) * 2.0f;
					Vec3f cameraSpaceAxis = Vec3f::cross(m_startArcballVector, currentArcballVector);	

					if (cameraSpaceAxis.length_sqr() > 1e-6f)
					{
						Vec4f worldAxis4 = m_camera.GetInverseViewMatrix() * Vec4f(cameraSpaceAxis.x, cameraSpaceAxis.y, cameraSpaceAxis.z, 0.0f);
						Vec3f worldAxis = Vec3f(worldAxis4.x, worldAxis4.y, worldAxis4.z).normalize();

						Mat4f rot = Mat4f::RotationAxis(worldAxis, angle);
						torus->m_rotationMatrix = rot * torus->m_baseRotationMatrix;
						Vec3f euler = Mat4f::ExtractEulerAngles(torus->m_rotationMatrix);
						torus->m_eulerAngles = { euler.x, euler.y, euler.z };
					}
				}
				else if (m_currentEditAction >= EditAction::RotateX && m_currentEditAction <= EditAction::RotateZ)
				{
					float angle = dx * 0.01f;
					Mat4f rot;

					if (m_currentEditAction == EditAction::RotateX) rot = Mat4f::RotationX(angle);
					else if (m_currentEditAction == EditAction::RotateY) rot = Mat4f::RotationY(angle);
					else if (m_currentEditAction == EditAction::RotateZ) rot = Mat4f::RotationZ(angle);

					torus->m_rotationMatrix = rot * torus->m_baseRotationMatrix;
					Vec3f euler = Mat4f::ExtractEulerAngles(torus->m_rotationMatrix);
					torus->m_eulerAngles = { euler.x, euler.y, euler.z };
				}
				else if (m_currentEditAction == EditAction::Scale)
				{
					float scaleFactor = 1.0f + dx * 0.01f;
					torus->SetScale(torus->GetScale() * scaleFactor);
				}
			}
			if (obj->type == ObjectType::Torus)
				static_cast<Torus*>(obj.get())->UpdateModelMatrix();

			if (m_currentEditAction < EditAction::RotateFree || m_currentEditAction > EditAction::RotateZ)
			{
				m_lastMousePos = { xPos, yPos };
			}

			return true;
		}


		if (m_interactionMode != InteractionMode::None)
		{
			if (m_interactionMode == InteractionMode::Orbiting)
			{
				m_camera.Orbit(dx * 0.01f, dy * 0.01f);
			}
			else if (m_interactionMode == InteractionMode::Panning)
			{
				float panSpeed = 0.002f * std::max(1.0f, m_camera.GetDistance());
				m_camera.Pan(-dx * panSpeed, dy * panSpeed);
			}

			UpdateProjectionMatrix();
			m_lastMousePos = { xPos, yPos };
		}
		return true;
	}
	case WM_MOUSEWHEEL:
	{
		short zDelta = (short)HIWORD(msg.wParam);
		m_camera.Zoom((zDelta / 120.0f) * 0.5f);
		UpdateProjectionMatrix();
		return true;
	}
	}

	return DxApplication::ProcessMessage(msg);
}

CadApplication::~CadApplication()
{
	Cursor3D::ReleaseSharedGeometry();

	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();
}

void CadApplication::UpdateResources(int width, int height)
{
	m_depthBuffer = m_device.CreateDepthStencilView(SIZE{ width, height });
	m_renderSize.cx = (width - cMenuWidth);
	m_renderSize.cy = height;

	Viewport viewport{ m_renderSize };
	m_device.getContext()->RSSetViewports(1, &viewport);


	UpdateProjectionMatrix();
	//m_viewMatrix = Mat4f::Identity();
	//if (m_cbPerPass)
	//{
	//	PerPassBuffer perPassData;
	//	perPassData.viewProj = m_projMatrix * m_viewMatrix;
	//	m_device.UpdateBuffer(m_cbPerPass, perPassData);
	//}
}

void CadApplication::Render()
{
	//SIZE wndSize = m_window.getClientSize();
	//int width = wndSize.cx;
	//int height = wndSize.cy;
	DrawMenu();

	//m_torus.UpdateMesh(m_device);

	auto& context = m_device.getContext();

	//const float clear_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	const float clear_color[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	context->ClearRenderTargetView(m_backBuffer.Get(), clear_color);
	context->ClearDepthStencilView(m_depthBuffer.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);

	context->OMSetRenderTargets(1, m_backBuffer.GetAddressOf(), m_depthBuffer.Get());

	context->VSSetConstantBuffers(0, 1, m_cbPerPass.GetAddressOf());

	context->IASetInputLayout(m_layout.Get());
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);
	context->VSSetConstantBuffers(1, 1, m_cbPerObject.GetAddressOf());

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);


	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, m_cursor.GetVertexBuffer().GetAddressOf(), &stride, &offset);
	DrawCursor(m_cursorPosition, 0.1f);
	auto selectionCenter = GetSelectionCenter();
	if (selectionCenter.has_value())
		DrawCursor(selectionCenter.value(), 0.06f);

	for (auto& obj : m_sceneObjects)
	{
		if (obj->type == ObjectType::Torus)
		{
			auto& torus = *static_cast<Torus*>(obj.get());
			torus.UpdateMesh(m_device);

			PerObjectBuffer objData;
			objData.model = torus.m_modelMatrix;
			objData.color = torus.selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, torus.GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->IASetIndexBuffer(torus.GetIndexBuffer().Get(), DXGI_FORMAT_R32_UINT, 0);
			context->DrawIndexed(static_cast<UINT>(torus.indices.size()), 0, 0);
		}
	}

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
	//context->VSSetShader(m_pointVertexShader.Get(), nullptr, 0);
	context->GSSetShader(m_pointGeometryShader.Get(), nullptr, 0);
	context->PSSetShader(m_pointPixelShader.Get(), nullptr, 0);

	for (auto& obj : m_sceneObjects)
	{
		if (obj->type == ObjectType::Point)
		{
			auto& point = *static_cast<Point*>(obj.get());
			PerObjectBuffer objData;
			objData.model = point.GetModelMatrix();
			objData.color = point.selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, point.GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->Draw(1, 0);
		}
	}
	context->GSSetShader(nullptr, nullptr, 0);

	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}


void CadApplication::DrawMenu()
{
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	//constexpr float menuWidth = 300.0f;
	ImGui::SetNextWindowPos(ImVec2(m_renderSize.cx, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(cMenuWidth, m_renderSize.cy));

	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse;

	ImGui::Begin("Menu", nullptr);

	if (m_menuState == MenuState::List)
	{
		if (ImGui::Button("Add Torus"))
		{
			m_sceneObjects.push_back(std::make_unique<Torus>(m_cursorPosition));
		}

		if (ImGui::Button("Add Point"))
		{
			m_sceneObjects.push_back(std::make_unique<Point>(m_cursorPosition));
		}

		ImGui::Separator();
		ImGuiIO& io = ImGui::GetIO();
		for (int i = 0; i < m_sceneObjects.size(); i++)
		{
			ImGui::PushID(i);
			auto& obj = m_sceneObjects[i];


			if (m_nameEditingIndex == i)
			{
				ImGui::SetKeyboardFocusHere();
				if (ImGui::InputText("##rename", m_renameBuffer, IM_ARRAYSIZE(m_renameBuffer),
					ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll))
				{

					obj->name = string(m_renameBuffer);
					m_nameEditingIndex = -1;
				}

				if (!ImGui::IsItemActive() && ImGui::IsMouseClicked(0))
				{
					m_nameEditingIndex = -1;
				}
			}
			else
			{
				if (ImGui::Selectable(obj->name.c_str(), obj->selected))
				{
					if (io.KeyCtrl && io.KeyShift)
					{
						if (m_lastClickedIndex != -1)
						{
							int start = std::min(i, m_lastClickedIndex);
							int end = std::max(i, m_lastClickedIndex);
							for (int j = start; j <= end; j++)
								m_sceneObjects[j]->selected = true;
						}
					}
					else if (io.KeyShift)
					{
						for (auto& o : m_sceneObjects)
							o->selected = false;
						if (m_lastClickedIndex != -1)
						{
							int start = std::min(i, m_lastClickedIndex);
							int end = std::max(i, m_lastClickedIndex);
							for (int j = start; j <= end; j++)
								m_sceneObjects[j]->selected = true;
						}
						else
						{
							obj->selected = true;
							m_lastClickedIndex = i;
						}
					}
					else if (io.KeyCtrl)
					{
						obj->selected = !obj->selected;
						m_lastClickedIndex = i;
					}
					else
					{
						for (auto& o : m_sceneObjects)
							o->selected = false;
						obj->selected = true;
						m_lastClickedIndex = i;
					}
				}

				if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0))
				{
					m_nameEditingIndex = i;
					strncpy_s(m_renameBuffer, obj->name.c_str(), sizeof(m_renameBuffer) - 1);
				}
			}

			ImGui::PopID();
		}

		ImGui::Separator();

		int selectedCount = 0;
		for (const auto& obj : m_sceneObjects)
			if (obj->selected) selectedCount++;

		ImGui::BeginDisabled(selectedCount != 1);
		if (ImGui::Button("Edit Selected", ImVec2(-1, 0)))
		{
			m_menuState = MenuState::Edit;
			m_currentEditAction = EditAction::None;
		}
		ImGui::EndDisabled();

		if (ImGui::Button("Select All", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
		{
			for (auto& obj : m_sceneObjects)
				obj->selected = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Deselect All", ImVec2(-1, 0)))
		{
			for (auto& obj : m_sceneObjects)
				obj->selected = false;
		}

		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

		if (ImGui::Button("Delete Selected", ImVec2(-1, 0)))
		{
			DeleteSelectedObjects();
		}

		ImGui::PopStyleColor(2);


		ImGui::Separator();
		ImGui::Text("Camera Settings");
		bool cameraChanged = false;
		if (ImGui::SliderFloat("FOV", &m_fovY, 30.0f, 120.0f))
			cameraChanged = true;
		if (ImGui::DragFloat("Near Plane", &m_nearPlane, 0.01f, 0.001f, 10.0f))
			cameraChanged = true;
		if (ImGui::DragFloat("Far Plane", &m_farPlane, 0.1f, 10.0f, 1000.0f))
			cameraChanged = true;

		if (cameraChanged)
			UpdateProjectionMatrix();


		ImGui::Separator();
		ImGui::Text("Cursor Settings");
		ImGui::DragFloat3("Cursor Position", &m_cursorPosition.x, 0.01f);
		Vec4f worldPos = Vec4f(m_cursorPosition.x, m_cursorPosition.y, m_cursorPosition.z, 1.0f);
		Vec4f clipPos = m_projViewMatrix * worldPos;
		int screenPos[2] = { 0 };
		if (std::abs(clipPos.w) > 0.0001f)
		{
			float ndcX = clipPos.x / clipPos.w;
			float ndcY = clipPos.y / clipPos.w;
			screenPos[0] = static_cast<int>(std::round((ndcX + 1.0f) * 0.5f * m_renderSize.cx));
			screenPos[1] = static_cast<int>(std::round((1.0f - ndcY) * 0.5f * m_renderSize.cy));
		}

		if (ImGui::DragInt2("Screen Position", screenPos))
		{
			screenPos[0] = std::clamp(screenPos[0], 0, static_cast<int>(m_renderSize.cx));
			screenPos[1] = std::clamp(screenPos[1], 0, static_cast<int>(m_renderSize.cy));
			auto [ndcX, ndcY] = CalculateCoordsFromPixel((float)screenPos[0], (float)screenPos[1], (float)m_renderSize.cx, (float)m_renderSize.cy);


			Vec4f viewPos = m_viewMatrix * worldPos;
			float actualDepth = std::abs(viewPos.z);

			float distance = m_camera.GetDistance();
			float fovY_rad = m_fovY * (std::numbers::pi_v<float> / 180.0f);
			float aspect = static_cast<float>(m_renderSize.cx) / m_renderSize.cy;

			float planeHeight = 2.0f * actualDepth * std::tan(fovY_rad / 2.0f);
			float planeWidth = planeHeight * aspect;

			Vec4f localPos{ ndcX * (planeWidth / 2.0f), ndcY * (planeHeight / 2.0f), viewPos.z, 1.0f };
			Vec4f newWorldPos = m_camera.GetInverseViewMatrix() * localPos;

			m_cursorPosition = { newWorldPos.x, newWorldPos.y, newWorldPos.z };
		}
	}
	else if (m_menuState == MenuState::Edit)
	{
		if (m_lastClickedIndex < 0 || m_lastClickedIndex >= m_sceneObjects.size())
		{
			m_menuState = MenuState::List;
		}
		else
		{
			auto& selectedObj = m_sceneObjects[m_lastClickedIndex];
			if (ImGui::Button("< Back to List"))
			{
				m_menuState = MenuState::List;
			}
			ImGui::Separator();

			ImGui::Text("Editing: %s", selectedObj->name.c_str());
			ImGui::Spacing();

			if (selectedObj->type == ObjectType::Point)
			{
				ImGui::DragFloat3("Position", &selectedObj->m_position.x, 0.01f);

				ImGui::Separator();
				ImGui::Text("Interactive Action");
				const char* actions[] = { "None", "Free Translation", "Translate X", "Translate Y", "Translate Z" };
				int actionIndex = static_cast<int>(m_currentEditAction);
				if (actionIndex > 4) actionIndex = 0; // Prevent out-of-bounds

				if (ImGui::Combo("##PointAction", &actionIndex, actions, IM_ARRAYSIZE(actions)))
				{
					m_currentEditAction = static_cast<EditAction>(actionIndex);
				}
			}
			else if (selectedObj->type == ObjectType::Torus)
			{
				auto torus = static_cast<Torus*>(selectedObj.get());
				bool transformChanged = false;

				// --- Geometry Settings ---
				float tempMajor = torus->GetMajorRadius();
				float tempMinor = torus->GetMinorRadius();
				int tempSegs[2] = { torus->GetMajorSegments(), torus->GetMinorSegments() };

				ImGui::Text("Geometry");
				if (ImGui::SliderFloat("Major Radius", &tempMajor, Torus::cMinMajorRadius, Torus::cMaxMajorRadius))
					torus->SetMajorRadius(tempMajor);
				if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, Torus::cMaxMinorRadius))
					torus->SetMinorRadius(tempMinor);
				if (ImGui::SliderInt2("Segments", tempSegs, Torus::cMinMajorSegments, Torus::cMaxMajorSegments))
					torus->SetSegments(tempSegs[0], tempSegs[1]);

				ImGui::Separator();

				if (ImGui::DragFloat3("Position", &torus->m_position.x, 0.01f))
					transformChanged = true;


				Vec3f eulerDegrees = Vec3f(torus->m_eulerAngles.x, torus->m_eulerAngles.y, torus->m_eulerAngles.z) * (180.0f / std::numbers::pi_v<float>);
				if (ImGui::DragFloat3("Rotation", eulerDegrees.f, 1.0f, 0.0f, 0.0f, "%.2f"))
				{
					eulerDegrees *= (std::numbers::pi_v<float> / 180.0f);
					torus->m_eulerAngles.x = eulerDegrees.x;
					torus->m_eulerAngles.y = eulerDegrees.y;
					torus->m_eulerAngles.z = eulerDegrees.z;
					MathLib::Mat4f rotX = MathLib::Mat4f::RotationX(torus->m_eulerAngles.x);
					MathLib::Mat4f rotY = MathLib::Mat4f::RotationY(torus->m_eulerAngles.y);
					MathLib::Mat4f rotZ = MathLib::Mat4f::RotationZ(torus->m_eulerAngles.z);
					torus->m_rotationMatrix = rotZ * rotX * rotY;
					transformChanged = true;
				}
				if (ImGui::Button("Reset Rotation"))
				{
					torus->m_eulerAngles = { 0, 0, 0 };
					torus->m_rotationMatrix = MathLib::Mat4f::Identity();
					torus->m_baseRotationMatrix = MathLib::Mat4f::Identity();
					transformChanged = true;
				}
				if (ImGui::DragFloat("Scale", &torus->m_scale, 0.01f, Torus::cMinScale, Torus::cMaxScale))
					transformChanged = true;

				if (transformChanged) 
					torus->UpdateModelMatrix();

				ImGui::Separator();

				ImGui::Text("Interactive Action");
				const char* actions[] = {
					"None", "Free Translation", "Translate X", "Translate Y", "Translate Z",
					"Free Arcball", "Rotate X", "Rotate Y", "Rotate Z", "Scale"
				};
				int actionIndex = static_cast<int>(m_currentEditAction);
				if (ImGui::Combo("##TorusAction", &actionIndex, actions, IM_ARRAYSIZE(actions)))
				{
					m_currentEditAction = static_cast<EditAction>(actionIndex);
				}
			}
		}
	}


	ImGui::End();
	ImGui::Render();
}