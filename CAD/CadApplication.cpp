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
		p = p.normalize();
	}

	return p;
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
	if (m_camera.UpdateProjectionMatrix())
		SyncPerPassBuffer();
}

void CadApplication::UpdateViewMatrix()
{
	if (m_camera.UpdateViewMatrices())
		SyncPerPassBuffer();
}

void CadApplication::SyncPerPassBuffer()
{
	if (m_cbPerPass)
	{
		PerPassBuffer perPassData;
		perPassData.viewProj = m_camera.GetProjViewMatrix();
		perPassData.aspectRatio = m_camera.GetAspectRatio();
		m_device.UpdateBuffer(m_cbPerPass, perPassData);
	}
}

void CadApplication::ClearSelection()
{
	for (auto& o : m_sceneObjects)
		o->selected = false;
	m_lastClickedIndex = std::nullopt;
	m_selectionCenterCache = std::nullopt;
	m_selectionDirty = false;
}

void CadApplication::HandleObjectSelection(size_t index, bool ctrlHeld, bool shiftHeld)
{
	if (ctrlHeld && shiftHeld)
	{
		if (m_lastClickedIndex.has_value())
		{
			size_t start = std::min(index, *m_lastClickedIndex);
			size_t end = std::max(index, *m_lastClickedIndex);
			for (size_t j = start; j <= end; j++)
				m_sceneObjects[j]->selected = true;
		}
	}
	else if (shiftHeld)
	{
		auto anchor = m_lastClickedIndex;
		ClearSelection();
		if (anchor.has_value())
		{
			size_t start = std::min(index, *anchor);
			size_t end = std::max(index, *anchor);
			for (size_t j = start; j <= end; j++)
				m_sceneObjects[j]->selected = true;
			m_lastClickedIndex = anchor;
		}
		else
		{
			m_sceneObjects[index]->selected = true;
			m_lastClickedIndex = index;
		}
	}
	else if (ctrlHeld)
	{
		m_sceneObjects[index]->selected = !m_sceneObjects[index]->selected;
		m_lastClickedIndex = index;
	}
	else
	{
		ClearSelection();
		m_sceneObjects[index]->selected = true;
		m_lastClickedIndex = index;
	}

	m_selectionDirty = true;
}

std::optional<size_t> CadApplication::PickClosestPoint(int mouseX, int mouseY, float toleranceSq)
{
	std::optional<size_t> closestIndex = std::nullopt;
	float minZ = std::numeric_limits<float>::max();

	for (size_t i = 0; i < m_sceneObjects.size(); i++)
	{
		const auto& obj = m_sceneObjects[i];
		if (obj->type != ObjectType::Point)
			continue;

		Vec4f worldPos = obj->m_position.ToVec4f(1.0f);
		Vec4f clipPos = m_camera.GetProjViewMatrix() * worldPos;

		if (clipPos.w <= 0.0f)
			continue;

		clipPos /= clipPos.w; // Perspective divide to get NDC

		float screenX = (clipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
		float screenY = (1.0f - clipPos.y) * 0.5f * m_renderSize.cy;
		float distSq = (screenX - mouseX) * (screenX - mouseX) + (screenY - mouseY) * (screenY - mouseY);

		if (distSq < toleranceSq && clipPos.z < minZ)
		{
			minZ = clipPos.z;
			closestIndex = i;
		}
	}

	return closestIndex;
}

void CadApplication::DrawCursor(float3 position, float scale)
{
	auto& context = m_device.getContext();

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
	m_lastClickedIndex = std::nullopt;
	m_selectionCenterCache = std::nullopt;
	m_selectionDirty = false;
}

std::optional<float3> CadApplication::GetSelectionCenter() const
{
	if (!m_selectionDirty)
		return m_selectionCenterCache;

	Vec4f sum; // .w counts the number of selected objects. Max possible count is 16 777 216 due to float precision.
	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected)
			sum += obj->m_position.ToVec4f(1.0f); //Vec4f(obj->m_position.x, obj->m_position.y, obj->m_position.z, 1.0f);
	}

	m_selectionCenterCache = sum.w < 1.0f ? std::nullopt : std::optional<float3>(float3::FromVec4f(sum / sum.w));
	m_selectionDirty = false;
	return m_selectionCenterCache;
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
		WORD fwKeys = LOWORD(msg.wParam);
		bool ctrlHeld = (fwKeys & MK_CONTROL) != 0;
		bool shiftHeld = (fwKeys & MK_SHIFT) != 0;

		if ((m_menuState == MenuState::Edit || m_menuState == MenuState::EditGroup) && m_currentEditAction != EditAction::None)
		{
			BeginEditAction(xPos, yPos, shiftHeld);
			return true;
		}

		auto pickedIndex = PickClosestPoint(xPos, yPos);
		if (pickedIndex.has_value())
		{
			HandleObjectSelection(*pickedIndex, ctrlHeld, false);
		}
		else
		{
			if (!ctrlHeld)
				ClearSelection();

			auto [normX, normY] = CalculateCoordsFromPixel(xPos, yPos, m_renderSize.cx, m_renderSize.cy);
			m_cursorPosition = m_camera.GetPositionOnFocalPlane(normX, normY);
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
	case WM_LBUTTONUP:
	case WM_RBUTTONUP:
	case WM_MBUTTONUP:
		m_isEditing = false;
		m_interactionMode = InteractionMode::None;
		ReleaseCapture();
		return true;
	case WM_MOUSEMOVE:
	{
		ApplyEditTransform(xPos, yPos);
		HandleCameraInteraction(xPos, yPos);
		return true;
	}
	case WM_MOUSEWHEEL:
	{
		short zDelta = (short)HIWORD(msg.wParam);
		m_camera.Zoom((zDelta / 120.0f) * 0.5f);
		UpdateViewMatrix();
		return true;
	}
	}

	return DxApplication::ProcessMessage(msg);
}

void CadApplication::HandleCameraInteraction(int mouseX, int mouseY)
{
	if (m_interactionMode == InteractionMode::None)
		return;

	int dx = mouseX - m_lastMousePos.x;
	int dy = mouseY - m_lastMousePos.y;
	if (m_interactionMode == InteractionMode::Orbiting)
		m_camera.Orbit(dx * 0.01f, dy * 0.01f);
	else if (m_interactionMode == InteractionMode::Panning)
	{
		float panSpeed = m_camera.GetPanScaleFactor() * m_camera.GetDistance();
		m_camera.Pan(-dx * panSpeed, dy * panSpeed);
	}

	UpdateViewMatrix();
	m_lastMousePos = { mouseX, mouseY };
}

void CadApplication::BeginEditAction(int mouseX, int mouseY, bool shiftHeld)
{
	m_isEditing = true;
	m_lastMousePos = { mouseX, mouseY };
	m_startMousePos = m_lastMousePos;

	if (shiftHeld)
		m_groupEditCenter = m_cursorPosition;
	else if (m_menuState == MenuState::Edit)
		m_groupEditCenter = m_sceneObjects[m_lastClickedIndex.value()]->m_position;
	else
	{
		auto center = GetSelectionCenter();
		m_groupEditCenter = center ? *center : float3(0.0f, 0.0f, 0.0f);
	}


	Vec4f pivotWorldPos = m_groupEditCenter.ToVec4f(1.0f);
	Vec4f pivotClipPos = m_camera.GetProjViewMatrix() * pivotWorldPos;
	bool isVisible = pivotClipPos.w > m_camera.GetNearPlane();

	if (isVisible)
	{
		pivotClipPos /= pivotClipPos.w;
		m_editObjScreenX = (pivotClipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
		m_editObjScreenY = (1.0f - pivotClipPos.y) * 0.5f * m_renderSize.cy;

		Vec3f toPivot = m_groupEditCenter.ToVec3f() - m_camera.GetPosition();
		m_editAnchorDepth = Vec3f::dot(toPivot, m_camera.GetForwardVector());
		m_startArcballVector = ScreenToObjectArcballVector(mouseX, mouseY, m_editObjScreenX, m_editObjScreenY, 150.0f);
	}
	else
	{
		m_editObjScreenX = m_renderSize.cx / 2.0f;
		m_editObjScreenY = m_renderSize.cy / 2.0f;
		m_editAnchorDepth = m_camera.GetNearPlane();
	}

	auto prepareObject = [](SceneObject* obj)
		{
			obj->m_basePosition = obj->m_position;
			if (obj->type == ObjectType::Torus)
			{
				auto torus = static_cast<Torus*>(obj);
				torus->m_baseRotationMatrix = torus->m_rotationMatrix;
				torus->m_baseScale = torus->m_scale;
			}
		};

	if (m_menuState == MenuState::Edit)
		prepareObject(m_sceneObjects[m_lastClickedIndex.value()].get());
	else
		for (auto& obj : m_sceneObjects)
			if (obj->selected)
				prepareObject(obj.get());

	SetCapture(m_window.getHandle());
}

void CadApplication::ApplyEditTransform(int mouseX, int mouseY)
{
	if (!m_isEditing)
		return;
	int totalDx = mouseX - m_startMousePos.x;
	int totalDy = mouseY - m_startMousePos.y;

	Vec3f editPivot = m_groupEditCenter.ToVec3f();
	Mat4f deltaRot = Mat4f::Identity();
	float scaleFactor = 1.0f;
	Vec3f worldDelta(0, 0, 0);
	bool isTranslating = false, isRotating = false, isScaling = false;

	if (m_currentEditAction >= EditAction::TranslateFree && m_currentEditAction <= EditAction::TranslateZ)
	{
		float moveX = totalDx * 0.01f;
		if (m_currentEditAction == EditAction::TranslateFree)
		{
			float unitsPerPixel = m_camera.GetPanScaleFactor() * std::abs(m_editAnchorDepth);
			worldDelta = (m_camera.GetRightVector() * totalDx - m_camera.GetUpVector() * totalDy) * unitsPerPixel;
		}
		else if (m_currentEditAction == EditAction::TranslateX) worldDelta.x = moveX;
		else if (m_currentEditAction == EditAction::TranslateY) worldDelta.y = moveX;
		else if (m_currentEditAction == EditAction::TranslateZ) worldDelta.z = moveX;

		isTranslating = true;
	}
	else if (m_currentEditAction == EditAction::RotateFree)
	{
		Vec3f currentArcballVector = ScreenToObjectArcballVector(mouseX, mouseY, m_editObjScreenX, m_editObjScreenY, 150.0f);
		float dot = std::clamp(Vec3f::dot(m_startArcballVector, currentArcballVector), -1.0f, 1.0f);
		float angle = std::acos(dot) * 2.0f;
		Vec3f cameraSpaceAxis = Vec3f::cross(m_startArcballVector, currentArcballVector);

		if (cameraSpaceAxis.length_sqr() > 1e-6f)
		{
			Vec4f worldAxis4 = m_camera.GetInverseViewMatrix() * Vec4f(cameraSpaceAxis.x, cameraSpaceAxis.y, cameraSpaceAxis.z, 0.0f);
			Vec3f worldAxis = Vec3f(worldAxis4.x, worldAxis4.y, worldAxis4.z).normalize();

			deltaRot = Mat4f::RotationAxis(worldAxis, angle);
			isRotating = true;
		}
	}
	else if (m_currentEditAction >= EditAction::RotateX && m_currentEditAction <= EditAction::RotateZ)
	{
		float angle = totalDx * 0.01f;
		if (m_currentEditAction == EditAction::RotateX) deltaRot = Mat4f::RotationX(angle);
		else if (m_currentEditAction == EditAction::RotateY) deltaRot = Mat4f::RotationY(angle);
		else if (m_currentEditAction == EditAction::RotateZ) deltaRot = Mat4f::RotationZ(angle);
		isRotating = true;
	}
	else if (m_currentEditAction == EditAction::Scale)
	{
		scaleFactor = std::max(0.01f, 1.0f + totalDx * 0.01f);
		isScaling = true;
	}

	if (isTranslating || isRotating || isScaling)
	{
		auto applyTransform = [&](const std::unique_ptr<SceneObject>& obj)
			{
				//Vec3f objBasePos = Vec3f(obj->m_basePosition.x, obj->m_basePosition.y, obj->m_basePosition.z);
				Vec3f objBasePos = obj->m_basePosition.ToVec3f();

				if (isTranslating)
				{
					Vec3f newPos = objBasePos + worldDelta;
					obj->m_position = newPos;
				}
				else if (isScaling)
				{
					Vec3f offset = objBasePos - editPivot;
					Vec3f newPos = editPivot + offset * scaleFactor;
					obj->m_position = newPos;

					if (obj->type == ObjectType::Torus)
					{
						auto torus = static_cast<Torus*>(obj.get());
						torus->SetScale(torus->m_baseScale * scaleFactor);
					}
				}
				else if (isRotating)
				{
					Vec3f offset = objBasePos - editPivot;
					Vec4f rotatedOffset4 = deltaRot * offset.ToVec4f(); //Vec4f(offset.x, offset.y, offset.z, 1.0f);
					//Vec3f rotatedOffset(rotatedOffset4.x, rotatedOffset4.y, rotatedOffset4.z);
					Vec3f rotatedOffset = Vec3f::FromVec4f(rotatedOffset4);
					Vec3f newPos = editPivot + rotatedOffset;

					obj->m_position = newPos;

					if (obj->type == ObjectType::Torus)
					{
						auto torus = static_cast<Torus*>(obj.get());
						torus->m_rotationMatrix = deltaRot * torus->m_baseRotationMatrix;
						Vec3f euler = Mat4f::ExtractEulerAngles(torus->m_rotationMatrix);
						torus->m_eulerAngles = euler;
					}
				}

				if (obj->type == ObjectType::Torus)
					static_cast<Torus*>(obj.get())->UpdateModelMatrix();
			};

		if (m_menuState == MenuState::EditGroup)
		{
			for (auto& obj : m_sceneObjects)
			{
				if (obj->selected)
					applyTransform(obj);
			}
		}
		else
		{
			applyTransform(m_sceneObjects[m_lastClickedIndex.value()]);
		}
		m_selectionDirty = true;
	}
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

	m_camera.SetViewportSize(m_renderSize.cx, m_renderSize.cy);
	UpdateProjectionMatrix();
}

void CadApplication::Render()
{
	DrawMenu();

	auto& context = m_device.getContext();

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
	if (selectionCenter)
		DrawCursor(*selectionCenter, 0.06f);

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

	ImGui::SetNextWindowPos(ImVec2(m_renderSize.cx, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(cMenuWidth, m_renderSize.cy));

	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse;

	ImGui::Begin("Menu", nullptr);


	int selectedCount = count_if(m_sceneObjects.cbegin(), m_sceneObjects.cend(), [](const auto& obj) { return obj->selected; });

	if (m_menuState == MenuState::List)
		DrawListMenu(selectedCount);
	else if (m_menuState == MenuState::Edit)
		DrawEditMenu();
	else if (m_menuState == MenuState::EditGroup)
		DrawEditGroupMenu(selectedCount);


	DrawCameraSettingsMenu();
	DrawCursorSettingsMenu();

	ImGui::End();
	ImGui::Render();
}

void CadApplication::DrawListMenu(int selectedCount)
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
				m_nameEditingIndex = std::nullopt;
			}

			if (!ImGui::IsItemActive() && ImGui::IsMouseClicked(0))
			{
				m_nameEditingIndex = std::nullopt;
			}
		}
		else
		{
			if (ImGui::Selectable(obj->name.c_str(), obj->selected))
			{
				HandleObjectSelection(i, io.KeyCtrl, io.KeyShift);
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


	ImGui::BeginDisabled(selectedCount == 0);
	if (selectedCount == 1)
	{
		if (ImGui::Button("Edit Selected", ImVec2(-1, 0)))
		{
			m_menuState = MenuState::Edit;
			m_currentEditAction = EditAction::None;
		}
	}
	else
	{
		if (ImGui::Button("Edit Group", ImVec2(-1, 0)))
		{
			m_menuState = MenuState::EditGroup;
			m_currentEditAction = EditAction::None;
		}
	}
	ImGui::EndDisabled();

	if (ImGui::Button("Select All", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (auto& obj : m_sceneObjects)
			obj->selected = true;
		m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All", ImVec2(-1, 0)))
	{
		ClearSelection();
	}

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Selected", ImVec2(-1, 0)))
	{
		DeleteSelectedObjects();
	}

	ImGui::PopStyleColor(2);

}

void CadApplication::DrawEditMenu()
{

	if (!m_lastClickedIndex || *m_lastClickedIndex >= m_sceneObjects.size())
	{
		m_menuState = MenuState::List;
	}
	else
	{
		auto& selectedObj = m_sceneObjects[*m_lastClickedIndex];
		if (ImGui::Button("< Back to List"))
		{
			m_menuState = MenuState::List;
		}
		ImGui::Separator();

		ImGui::Text("Editing: %s", selectedObj->name.c_str());
		ImGui::Spacing();

		if (selectedObj->type == ObjectType::Point)
		{
			DrawPointMenu(*selectedObj);
		}
		else if (selectedObj->type == ObjectType::Torus)
		{
			DrawTorusMenu(*static_cast<Torus*>(selectedObj.get()));
		}
	}

}

void CadApplication::DrawTorusMenu(Torus& torus)
{
	bool transformChanged = false;

	// --- Geometry Settings ---
	float tempMajor = torus.GetMajorRadius();
	float tempMinor = torus.GetMinorRadius();
	int tempSegs[2] = { torus.GetMajorSegments(), torus.GetMinorSegments() };

	ImGui::Text("Geometry");
	if (ImGui::SliderFloat("Major Radius", &tempMajor, Torus::cMinMajorRadius, Torus::cMaxMajorRadius))
		torus.SetMajorRadius(tempMajor);
	if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, Torus::cMaxMinorRadius))
		torus.SetMinorRadius(tempMinor);
	if (ImGui::SliderInt2("Segments", tempSegs, Torus::cMinMajorSegments, Torus::cMaxMajorSegments))
		torus.SetSegments(tempSegs[0], tempSegs[1]);

	ImGui::Separator();

	if (ImGui::DragFloat3("Position", &torus.m_position.x, 0.01f))
	{
		transformChanged = true;
		m_selectionDirty = true;
	}

	//Vec3f eulerDegrees = Vec3f(torus.m_eulerAngles.x, torus.m_eulerAngles.y, torus.m_eulerAngles.z) * (180.0f / std::numbers::pi_v<float>);
	Vec3f eulerDegrees = torus.m_eulerAngles.ToVec3f() * (180.0f / std::numbers::pi_v<float>);
	if (ImGui::DragFloat3("Rotation", eulerDegrees.f, 1.0f, 0.0f, 0.0f, "%.2f"))
	{
		eulerDegrees *= (std::numbers::pi_v<float> / 180.0f);
		torus.m_eulerAngles = eulerDegrees;
		//torus.m_eulerAngles.x = eulerDegrees.x;
		//torus.m_eulerAngles.y = eulerDegrees.y;
		//torus.m_eulerAngles.z = eulerDegrees.z;
		MathLib::Mat4f rotX = MathLib::Mat4f::RotationX(torus.m_eulerAngles.x);
		MathLib::Mat4f rotY = MathLib::Mat4f::RotationY(torus.m_eulerAngles.y);
		MathLib::Mat4f rotZ = MathLib::Mat4f::RotationZ(torus.m_eulerAngles.z);
		torus.m_rotationMatrix = rotZ * rotX * rotY;
		transformChanged = true;
	}
	if (ImGui::Button("Reset Rotation"))
	{
		torus.m_eulerAngles = { 0, 0, 0 };
		torus.m_rotationMatrix = MathLib::Mat4f::Identity();
		torus.m_baseRotationMatrix = MathLib::Mat4f::Identity();
		transformChanged = true;
	}
	if (ImGui::DragFloat("Scale", &torus.m_scale, 0.01f, Torus::cMinScale, Torus::cMaxScale))
		transformChanged = true;

	if (transformChanged)
		torus.UpdateModelMatrix();

	ImGui::Separator();
	DrawActionCombo();
}

void CadApplication::DrawPointMenu(SceneObject& selectedObj)
{
	if (ImGui::DragFloat3("Position", &selectedObj.m_position.x, 0.01f))
		m_selectionDirty = true;

	ImGui::Separator();
	DrawActionCombo();
}

void CadApplication::DrawActionCombo()
{
	ImGui::Text("Interactive Action");
	const char* actions[] = {
		"None", "Free Translation", "Translate X", "Translate Y", "Translate Z",
		"Free Arcball", "Rotate X", "Rotate Y", "Rotate Z", "Scale"
	};
	int actionIndex = static_cast<int>(m_currentEditAction);

	if (ImGui::Combo("##ObjectAction", &actionIndex, actions, IM_ARRAYSIZE(actions)))
	{
		m_currentEditAction = static_cast<EditAction>(actionIndex);
	}
}

void CadApplication::DrawEditGroupMenu(int selectedCount)
{

	if (ImGui::Button("< Back to List"))
	{
		m_menuState = MenuState::List;
	}

	ImGui::Separator();

	ImGui::Text("Editing Group (%d objects)", selectedCount);

	auto centerOpt = GetSelectionCenter();
	if (centerOpt)
	{
		ImGui::Text("Center: %.2f, %.2f, %.2f", centerOpt->x, centerOpt->y, centerOpt->z);
	}
	ImGui::Spacing();
	DrawActionCombo();
	//ImGui::Text("Interactive Group Action");
	//const char* actions[] = {
	//	"None", "Free Translation", "Translate X", "Translate Y", "Translate Z",
	//	"Free Arcball", "Rotate X", "Rotate Y", "Rotate Z", "Scale"
	//};
	//int actionIndex = static_cast<int>(m_currentEditAction);
	//if (ImGui::Combo("##GroupAction", &actionIndex, actions, IM_ARRAYSIZE(actions)))
	//{
	//	m_currentEditAction = static_cast<EditAction>(actionIndex);
	//}
}

void CadApplication::DrawCameraSettingsMenu()
{
	ImGui::Separator();
	ImGui::Text("Camera Settings");
	float tempFovDegrees = m_camera.GetFovY() * (180.0f / std::numbers::pi_v<float>);
	float tempNear = m_camera.GetNearPlane();
	float tempFar = m_camera.GetFarPlane();

	if (ImGui::SliderFloat("FOV", &tempFovDegrees, 30.0f, 120.0f))
		m_camera.SetFovY(tempFovDegrees * (std::numbers::pi_v<float> / 180.0f));

	if (ImGui::DragFloat("Near Plane", &tempNear, 0.01f, 0.001f, 10.0f) ||
		ImGui::DragFloat("Far Plane", &tempFar, 0.1f, 10.0f, 1000.0f))
		m_camera.SetPlanes(tempNear, tempFar);

	UpdateProjectionMatrix();
}

void CadApplication::DrawCursorSettingsMenu()
{
	ImGui::Separator();
	ImGui::Text("Cursor Settings");
	ImGui::DragFloat3("Cursor Position", &m_cursorPosition.x, 0.01f);
	Vec4f worldPos = Vec4f(m_cursorPosition.x, m_cursorPosition.y, m_cursorPosition.z, 1.0f);
	Vec4f clipPos = m_camera.GetProjViewMatrix() * worldPos;
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
		auto [ndcX, ndcY] = CalculateCoordsFromPixel(screenPos[0], screenPos[1], m_renderSize.cx, m_renderSize.cy);
		Vec4f worldPos = m_cursorPosition.ToVec4f(1.0f);
		Vec4f viewPos = m_camera.GetViewMatrix() * worldPos;
		float actualDepth = std::abs(viewPos.z);

		m_cursorPosition = m_camera.GetPositionAtDepth(ndcX, ndcY, actualDepth);
	}
}
