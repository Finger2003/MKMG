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
	const auto bezierVsByteCode = DxDevice::LoadByteCode(L"BezierVS.cso");
	const auto bezierGsByteCode = DxDevice::LoadByteCode(L"BezierGS.cso");
	const auto surfaceVsByteCode = DxDevice::LoadByteCode(L"SurfaceVS.cso");
	const auto surfaceHsByteCode = DxDevice::LoadByteCode(L"SurfaceHS.cso");
	const auto surfaceDsByteCode = DxDevice::LoadByteCode(L"SurfaceDS.cso");

	m_vertexShader = m_device.CreateVertexShader(vsByteCode);
	m_pixelShader = m_device.CreatePixelShader(psByteCode);
	m_pointVertexShader = m_device.CreateVertexShader(pointVsByteCode);
	m_pointPixelShader = m_device.CreatePixelShader(pointPsByteCode);
	m_pointGeometryShader = m_device.CreateGeometryShader(pointGsByteCode);
	m_bezierVertexShader = m_device.CreateVertexShader(bezierVsByteCode);
	m_bezierGeometryShader = m_device.CreateGeometryShader(bezierGsByteCode);
	m_surfaceVertexShader = m_device.CreateVertexShader(surfaceVsByteCode);
	m_surfaceDomainShader = m_device.CreateDomainShader(surfaceDsByteCode);
	m_surfaceHullShader = m_device.CreateHullShader(surfaceHsByteCode);

	vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	m_layout = m_device.CreateInputLayout(inputElements, vsByteCode);


	// Create the constant buffers for MVP matrices
	m_cbPerPass = m_device.CreateConstantBuffer<PerPassBuffer>();
	m_cbPerObject = m_device.CreateConstantBuffer<PerObjectBuffer>();
	InitStereoBlendStates();
	InitImGui();


	Cursor3D::InitSharedGeometry(m_device);
}

std::optional<VirtualPointMapping> CadApplication::PickVirtualPoint(int mouseX, int mouseY, float toleranceSq)
{
	std::optional<VirtualPointMapping> closestMapping = std::nullopt;
	float minZ = std::numeric_limits<float>::max();

	for (const auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;

		if (auto curve = obj->As<BSplineCurve>())
		{
			//auto curve = static_cast<BSplineCurve*>(obj.get());
			for (const auto& mapping : curve->m_virtualPoints)
			{
				Vec4f worldPos = mapping.virtualPosition.ToVec4f(1.0f);
				Vec4f clipPos = m_camera.GetProjViewMatrix() * worldPos;
				if (clipPos.w <= 0.0f)
					continue;
				clipPos /= clipPos.w;

				float screenX = (clipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
				float screenY = (1.0f - clipPos.y) * 0.5f * m_renderSize.cy;
				float distSq = (screenX - mouseX) * (screenX - mouseX) + (screenY - mouseY) * (screenY - mouseY);

				if (distSq < toleranceSq && clipPos.z < minZ)
				{
					minZ = clipPos.z;
					closestMapping = mapping;
				}
			}
		}
	}

	return closestMapping;
}

void CadApplication::BeginVirtualEditAction(int mouseX, int mouseY, const VirtualPointMapping& mapping)
{
	m_lastMousePos = { mouseX, mouseY };
	m_startMousePos = m_lastMousePos;

	Vec4f pivotWorldPos = mapping.virtualPosition.ToVec4f(1.0f);
	Vec4f pivotClipPos = m_camera.GetProjViewMatrix() * pivotWorldPos;

	if (pivotClipPos.w > 0.0f)
	{
		Vec3f toPivot = mapping.virtualPosition.ToVec3f() - m_camera.GetPosition();
		m_editAnchorDepth = Vec3f::dot(toPivot, m_camera.GetForwardVector());
	}

	if (auto pt = mapping.targetPoint.lock())
		pt->m_basePosition = pt->m_position;

	SetCapture(m_window.getHandle());
}

void CadApplication::ApplyVirtualEditTransform(int mouseX, int mouseY)
{
	if (!m_activeVirtualEdit.has_value()) return;

	int totalDx = mouseX - m_startMousePos.x;
	int totalDy = mouseY - m_startMousePos.y;

	float unitsPerPixel = m_camera.GetPanScaleFactor() * std::abs(m_editAnchorDepth);
	Vec3f worldDelta = (m_camera.GetRightVector() * totalDx - m_camera.GetUpVector() * totalDy) * unitsPerPixel;

	worldDelta = worldDelta * (1.0f / m_activeVirtualEdit->weight);

	if (auto pt = m_activeVirtualEdit->targetPoint.lock())
	{
		pt->m_position = pt->m_basePosition.ToVec3f() + worldDelta;
	}
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
		perPassData.renderSize[0] = static_cast<float>(m_renderSize.cx);
		perPassData.renderSize[1] = static_cast<float>(m_renderSize.cy);
		perPassData.stereoTint = { 1.0f, 1.0f, 1.0f, 1.0f };
		m_device.UpdateBuffer(m_cbPerPass, perPassData);
	}
}

void CadApplication::ClearSelection()
{
	for (auto& o : m_sceneObjects)
		o->selected = false;
	m_lastClickedIndex = std::nullopt;
	m_lastCurveClickedIndex = std::nullopt;
	m_selectionCenterCache = std::nullopt;
	m_selectionDirty = false;
}

void CadApplication::HandleObjectSelection(size_t index, bool ctrlHeld, bool shiftHeld)
{
	auto setSelection = [&](size_t i, bool state) {
		m_sceneObjects[i]->selected = state;

		if (auto curve = m_sceneObjects[i]->As<Curve>())
		{
			for (const auto& cpWeak : curve->m_controlPoints)
			{
				if (auto cp = cpWeak.lock())
					cp->selected = state;
			}
		}
		else if (auto surface = m_sceneObjects[i]->As<Surface>())
		{
			for (const auto& cpWeak : surface->m_controlPoints)
			{
				if (auto cp = cpWeak.lock())
					cp->selected = state;
			}
		}		
		};

	if (ctrlHeld && shiftHeld)
	{
		if (m_lastClickedIndex.has_value())
		{
			size_t start = std::min(index, *m_lastClickedIndex);
			size_t end = std::max(index, *m_lastClickedIndex);
			for (size_t j = start; j <= end; j++)
				setSelection(j, true);
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
				setSelection(j, true);
			m_lastClickedIndex = anchor;
		}
		else
		{
			setSelection(index, true);
			m_lastClickedIndex = index;
		}
	}
	else if (ctrlHeld)
	{
		setSelection(index, !m_sceneObjects[index]->selected);
		m_lastClickedIndex = index;
	}
	else
	{
		ClearSelection();
		setSelection(index, true);
		m_lastClickedIndex = index;
	}


	m_selectionDirty = true;
}

void CadApplication::HandleCurveListSelection(Curve* curve, size_t index, bool ctrlHeld, bool shiftHeld)
{
	auto setSelection = [&](size_t i, bool state) {
		if (auto cp = curve->m_controlPoints[i].lock())
			cp->selected = state;
		};

	if (ctrlHeld && shiftHeld)
	{
		if (m_lastCurveClickedIndex.has_value())
		{
			size_t start = std::min(index, *m_lastCurveClickedIndex);
			size_t end = std::max(index, *m_lastCurveClickedIndex);
			for (size_t j = start; j <= end; j++)
				setSelection(j, true);
		}
	}
	else if (shiftHeld)
	{
		auto anchor = m_lastCurveClickedIndex;
		if (anchor.has_value())
		{
			size_t start = std::min(index, *anchor);
			size_t end = std::max(index, *anchor);
			for (size_t j = start; j <= end; j++)
				setSelection(j, true);
			m_lastCurveClickedIndex = anchor;
		}
		else
		{
			setSelection(index, true);
			m_lastCurveClickedIndex = index;
		}
	}
	else if (ctrlHeld)
	{
		if (auto cp = curve->m_controlPoints[index].lock())
			cp->selected = !cp->selected;
		m_lastCurveClickedIndex = index;
	}
	else
	{
		for (size_t i = 0; i < curve->m_controlPoints.size(); i++)
			setSelection(i, i == index);
		m_lastCurveClickedIndex = index;
	}

	m_selectionDirty = true;
}

std::optional<size_t> CadApplication::PickClosestPoint(int mouseX, int mouseY, float toleranceSq)
{
	std::optional<size_t> closestIndex = std::nullopt;
	float minZ = std::numeric_limits<float>::max();

	for (size_t i = 0; i < m_sceneObjects.size(); i++)
	{		
		if (const auto point = m_sceneObjects[i]->As<Point>())
		{
			Vec4f worldPos = point->m_position.ToVec4f(1.0f);
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
	for (auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;

		if (auto surface = obj->As<Surface>())
		{
			for (const auto& cpWeak : surface->m_controlPoints)
			{
				if (auto cp = cpWeak.lock())
					cp->isLockedToSurface = false;
			}
		}		
	}

	erase_if(m_sceneObjects, [](const auto& obj) {
		if (!obj->selected)
			return false;

		if (auto pt = obj->As<Point>())
		{
			if (pt->isLockedToSurface)
				return false;
		}

		return true;
		});
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
		if (!obj->selected)
			continue;

		if (const auto transObj = obj->As<TransformableObject>())
			sum += transObj->m_position.ToVec4f(1.0f);
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
		bool pPressed = (GetAsyncKeyState('P') & 0x8000) != 0;

		if (m_enableVirtualEdit)
		{
			m_activeVirtualEdit = PickVirtualPoint(xPos, yPos);
			if (m_activeVirtualEdit.has_value())
				BeginVirtualEditAction(xPos, yPos, *m_activeVirtualEdit);
			return true;
		}

		if (pPressed)
		{
			auto [normX, normY] = CalculateCoordsFromPixel(static_cast<float>(xPos), static_cast<float>(yPos),
				static_cast<float>(m_renderSize.cx), static_cast<float>(m_renderSize.cy));
			m_cursorPosition = m_camera.GetPositionOnFocalPlane(normX, normY);

			auto newPoint = std::make_shared<Point>(m_cursorPosition);
			m_sceneObjects.push_back(newPoint);

			std::shared_ptr<Curve> activeCurve;
			int selectedCurves = 0;
			for (const auto& obj : m_sceneObjects)
			{
				if (obj->selected && obj->IsA(ObjectType::Curve))
				{
					selectedCurves++;
					activeCurve = std::static_pointer_cast<Curve>(obj);
				}
			}

			if (selectedCurves == 1 && activeCurve)
				activeCurve->m_controlPoints.push_back(newPoint);

			return true;
		}

		auto pickedIndex = PickClosestPoint(xPos, yPos);
		if (pickedIndex.has_value())
		{
			HandleObjectSelection(*pickedIndex, ctrlHeld, false);

			if (m_currentEditAction != EditAction::None)
				BeginEditAction(xPos, yPos, shiftHeld);
		}
		else
		{
			if (ctrlHeld)
			{
				auto [normX, normY] = CalculateCoordsFromPixel(static_cast<float>(xPos), static_cast<float>(yPos),
					static_cast<float>(m_renderSize.cx), static_cast<float>(m_renderSize.cy));
				m_cursorPosition = m_camera.GetPositionOnFocalPlane(normX, normY);
			}
			else if (m_currentEditAction != EditAction::None)
				BeginEditAction(xPos, yPos, shiftHeld);
			else
			{
				ClearSelection();
				auto [normX, normY] = CalculateCoordsFromPixel(static_cast<float>(xPos), static_cast<float>(yPos),
					static_cast<float>(m_renderSize.cx), static_cast<float>(m_renderSize.cy));
				m_cursorPosition = m_camera.GetPositionOnFocalPlane(normX, normY);
			}
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
		m_activeVirtualEdit = std::nullopt;
		m_interactionMode = InteractionMode::None;
		ReleaseCapture();
		return true;
	case WM_MOUSEMOVE:
	{
		if (m_enableVirtualEdit)
			ApplyVirtualEditTransform(xPos, yPos);
		else
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

	TransformableObject* editObj = nullptr;
	if (m_menuState == MenuState::Edit)
	{
		if (!m_lastClickedIndex.has_value())
			return;

		editObj = m_sceneObjects[*m_lastClickedIndex]->As<TransformableObject>();
		if (!editObj)
			return;
	}

	if (shiftHeld)
		m_groupEditCenter = m_cursorPosition;
	else if (m_menuState == MenuState::Edit)
		m_groupEditCenter = editObj->m_position;
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

	auto prepareObject = [](TransformableObject* obj)
		{
			obj->m_basePosition = obj->m_position;
			if (auto torus = obj->As<Torus>())
			{
				torus->m_baseRotationMatrix = torus->m_rotationMatrix;
				torus->m_baseScale = torus->m_scale;
			}
		};

	if (m_menuState == MenuState::Edit)
		prepareObject(editObj);
	else
		for (auto& obj : m_sceneObjects)
		{
			if (!obj->selected)
				continue;

			if (auto transObj = obj->As<TransformableObject>())
				prepareObject(transObj);
		}

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
		auto applyTransform = [&](TransformableObject* obj)
			{
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

					if (auto torus = obj->As<Torus>())
						torus->SetScale(torus->m_baseScale * scaleFactor);
				}
				else if (isRotating)
				{
					Vec3f offset = objBasePos - editPivot;
					Vec4f rotatedOffset4 = deltaRot * offset.ToVec4f();
					Vec3f rotatedOffset = Vec3f::FromVec4f(rotatedOffset4);
					Vec3f newPos = editPivot + rotatedOffset;

					obj->m_position = newPos;


					if (auto torus = obj->As<Torus>())
					{
						torus->m_rotationMatrix = deltaRot * torus->m_baseRotationMatrix;
						Vec3f euler = Mat4f::ExtractEulerAngles(torus->m_rotationMatrix);
						torus->m_eulerAngles = euler;
					}
				}

				if (auto torus = obj->As<Torus>())
					torus->UpdateModelMatrix();
				if (auto pt = obj->As<Point>())
					pt->NotifyDependents();
			};


		if (m_menuState == MenuState::Edit)
		{
			if (m_lastClickedIndex)
			{
				if (auto transObj = m_sceneObjects[*m_lastClickedIndex]->As<TransformableObject>())
					applyTransform(transObj);
			}
		}
		else
		{
			for (auto& obj : m_sceneObjects)
			{
				if (!obj->selected)
					continue;
				if (auto transObj = obj->As<TransformableObject>())
					applyTransform(transObj);
			}
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
	context->OMSetRenderTargets(1, m_backBuffer.GetAddressOf(), m_depthBuffer.Get());

	if (!m_enableStereo)
	{
		context->OMSetBlendState(m_blendStateDefault.Get(), nullptr, 0xFFFFFFFF);
		context->ClearDepthStencilView(m_depthBuffer.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);
		SyncPerPassBuffer();
		DrawScene(context);
	}
	else
	{
		context->OMSetBlendState(m_blendStateAnaglyph.Get(), nullptr, 0xFFFFFFFF);

		// Left eye
		context->ClearDepthStencilView(m_depthBuffer.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);
		SetupStereoCamera(true, m_leftEyeColor);
		DrawScene(context);

		// Right eye
		context->ClearDepthStencilView(m_depthBuffer.Get(), D3D11_CLEAR_DEPTH, 1.0f, 0);
		SetupStereoCamera(false, m_rightEyeColor);
		DrawScene(context);
		context->OMSetBlendState(m_blendStateDefault.Get(), nullptr, 0xFFFFFFFF);
	}

	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}

void CadApplication::DrawPoints(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	for (auto& obj : m_sceneObjects)
	{
		if (auto point = obj->As<Point>())
		{
			PerObjectBuffer objData;
			objData.model = point->GetModelMatrix();
			objData.color = point->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, point->GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->Draw(1, 0);
		}
	}

	if (m_showSurfacePopup)
	{
		for (auto& point : m_previewPoints)
		{
			PerObjectBuffer objData;
			objData.model = point->GetModelMatrix();
			objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, point->GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->Draw(1, 0);
		}
	}
}

void CadApplication::DrawPolylines(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.7f, 0.7f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : m_sceneObjects)
	{
		if (auto curve = obj->As<Curve>())
		{
			curve->UpdatePolyline(m_device);
			if (curve->selected)
			{
				ID3D11Buffer** bufferToDraw = curve->m_lineVertexBuffer.GetAddressOf();
				UINT countToDraw = curve->m_lineVertexCount;

				if (m_showBernsteinPoints)
				{
					if (auto bsplineCurve = curve->As<BSplineCurve>())
					{
						bufferToDraw = bsplineCurve->m_bernsteinVertexBuffer.GetAddressOf();
						countToDraw = bsplineCurve->m_bernsteinVertexCount;
					}
				}

				if (countToDraw > 0)
				{
					UINT stride = sizeof(VertexPosition);
					UINT offset = 0;
					context->IASetVertexBuffers(0, 1, bufferToDraw, &stride, &offset);
					context->Draw(countToDraw, 0);
				}
			}
		}
	}
}

void CadApplication::DrawBezierCurves(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();

	for (auto& obj : m_sceneObjects)
	{
		if (auto curve = obj->As<Curve>())
		{
			objData.color = curve->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			if (curve->m_curveVertexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, curve->m_curveVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(curve->m_curveVertexCount, 0);
			}
		}
	}
}

void CadApplication::DrawVirtualBernsteinPoints(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	if (!m_showBernsteinPoints) return;

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;

		if (auto bspline = obj->As<BSplineCurve>())
		{
			if (bspline->m_bernsteinVertexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, bspline->m_bernsteinVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(bspline->m_bernsteinVertexCount, 0);
			}
		}
	}
}

void CadApplication::DrawSurfaces(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{

	if (m_showSurfacePopup && m_previewSurface)
		DrawSurface(context, m_previewSurface.get(), Vec4f(0.0f, 0.5f, 1.0f, 1.0f));	

	for (auto& obj : m_sceneObjects)
	{
		if (auto surface = obj->As<Surface>())
			DrawSurface(context, surface, surface->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
	}
}

void CadApplication::DrawSurface(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context, Surface* surface, MathLib::Vec4f color)
{
	surface->UpdateVertices(m_device);
	if (surface->m_patchIndexCount > 0)
	{
		UINT stride = sizeof(VertexPosition);
		UINT offset = 0;
		context->IASetVertexBuffers(0, 1, surface->m_patchVertexBuffer.GetAddressOf(), &stride, &offset);
		context->IASetIndexBuffer(surface->m_patchIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
		PerObjectBuffer objData;
		objData.color = color;

		auto updateAndDraw = [&](Surface* surf, float xParam, float yParam) {
			objData.surfaceParams =
			{
				xParam,
				yParam,
				static_cast<float>(surf->m_smoothness),
				0.0f
			};

			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->DrawIndexed(surf->m_patchIndexCount, 0, 0);
			};

		updateAndDraw(surface, 0.0f, static_cast<float>(surface->m_linesPerSegmentU));
		updateAndDraw(surface, 1.0f, static_cast<float>(surface->m_linesPerSegmentV));
	}
}

void CadApplication::DrawSurfacesPolylines(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.7f, 0.7f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : m_sceneObjects)
	{
		if (auto surface = obj->As<Surface>())
		{
			surface->UpdateVertices(m_device);
			if (surface->selected && surface->m_polylineIndexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, surface->m_polylineVertexBuffer.GetAddressOf(), &stride, &offset);
				context->IASetIndexBuffer(surface->m_polylineIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
				context->DrawIndexed(surface->m_polylineIndexCount, 0, 0);
			}
		}		
	}
}

void CadApplication::InitStereoBlendStates()
{
	m_blendStateDefault = m_device.CreateBlendState();
	m_blendStateAnaglyph = m_device.CreateBlendState(BlendDescription::MaxBlendDescription());
}

void CadApplication::SetupStereoCamera(bool isLeftEye, const MathLib::Vec4f& eyeTint)
{
	float E = m_eyeSeparation * 0.5f;
	float shiftMultiplier = isLeftEye ? 1.0f : -1.0f;
	float E_shift = E * shiftMultiplier;

	Mat4f stereoView = Mat4f::Translation(E_shift, 0.0f, 0.0f) * m_camera.GetViewMatrix();
	float aspect = m_camera.GetAspectRatio();
	float fov = m_camera.GetFovY();
	float n = m_camera.GetNearPlane();
	float f = m_camera.GetFarPlane();

	float top = n * std::tan(fov / 2.0f);
	float bottom = -top;

	float frustumShift = E_shift * (n / m_focalLength);
	float width = aspect * top;

	float left = -width + frustumShift;
	float right = width + frustumShift;

	Mat4f stereoProj = Mat4f::Frustum(left, right, bottom, top, n, f);

	PerPassBuffer perPassData;
	perPassData.viewProj = stereoProj * stereoView;
	perPassData.aspectRatio = aspect;
	perPassData.renderSize[0] = static_cast<float>(m_renderSize.cx);
	perPassData.renderSize[1] = static_cast<float>(m_renderSize.cy);
	perPassData.stereoTint = eyeTint;
	m_device.UpdateBuffer(m_cbPerPass, perPassData);
}

void CadApplication::DrawScene(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	ID3D11Buffer* buffers[] = { m_cbPerPass.Get(), m_cbPerObject.Get() };
	context->VSSetConstantBuffers(0, 2, buffers);
	context->HSSetConstantBuffers(0, 2, buffers);
	context->DSSetConstantBuffers(0, 2, buffers);
	context->GSSetConstantBuffers(0, 2, buffers);
	context->PSSetConstantBuffers(0, 2, buffers);

	context->IASetInputLayout(m_layout.Get());
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

	DrawCursors(context);
	DrawToruses(context);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP);
	DrawPolylines(context);
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	DrawSurfacesPolylines(context);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST_ADJ);
	context->VSSetShader(m_bezierVertexShader.Get(), nullptr, 0);
	context->GSSetShader(m_bezierGeometryShader.Get(), nullptr, 0);
	DrawBezierCurves(context);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->GSSetShader(m_pointGeometryShader.Get(), nullptr, 0);
	context->PSSetShader(m_pointPixelShader.Get(), nullptr, 0);

	DrawPoints(context);
	DrawVirtualBernsteinPoints(context);

	context->GSSetShader(nullptr, nullptr, 0);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_16_CONTROL_POINT_PATCHLIST);

	context->VSSetShader(m_surfaceVertexShader.Get(), nullptr, 0);
	context->HSSetShader(m_surfaceHullShader.Get(), nullptr, 0);
	context->DSSetShader(m_surfaceDomainShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);

	DrawSurfaces(context);

	context->HSSetShader(nullptr, nullptr, 0);
	context->DSSetShader(nullptr, nullptr, 0);
}

SurfaceGenerationResult CadApplication::GenerateSurface() const
{
	const auto shape = static_cast<SurfaceShape>(m_previewShape);
	if (m_previewType == 0)
	{
		if (shape == SurfaceShape::Flat)
			return BezierSurface::CreateFlat(m_previewSegU, m_previewSegV, m_previewWidth, m_previewDim2, m_cursorPosition, m_device);
		else
			return BezierSurface::CreateCylinder(m_previewSegU, m_previewSegV, m_previewRadius, m_previewDim2, m_cursorPosition, m_device);
	}
	else if (m_previewType == 1)
	{
		if (shape == SurfaceShape::Flat)
			return BSplineSurface::CreateFlat(m_previewSegU, m_previewSegV, m_previewWidth, m_previewDim2, m_cursorPosition, m_device);
		else
			return BSplineSurface::CreateCylinder(m_previewSegU, m_previewSegV, m_previewRadius, m_previewDim2, m_cursorPosition, m_device);
	}	
}

bool CadApplication::ContainsCaseInsensitive(const std::string& str, const std::string& substr)
{
	auto it = std::search(
		str.begin(), str.end(),
		substr.begin(), substr.end(),
		[](char ch1, char ch2) { return std::tolower(ch1) == std::tolower(ch2); }
	);
	return (it != str.end());
		
}

void CadApplication::DrawToruses(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	for (auto& obj : m_sceneObjects)
	{
		if (auto torus = obj->As<Torus>())
		{
			torus->UpdateMesh(m_device);

			PerObjectBuffer objData;
			objData.model = torus->m_modelMatrix;
			objData.color = torus->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, torus->GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->IASetIndexBuffer(torus->GetIndexBuffer().Get(), DXGI_FORMAT_R32_UINT, 0);
			context->DrawIndexed(static_cast<UINT>(torus->indices.size()), 0, 0);
		}
	}
}

void CadApplication::DrawCursors(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, m_cursor.GetVertexBuffer().GetAddressOf(), &stride, &offset);
	DrawCursor(m_cursorPosition, 0.1f);
	auto selectionCenter = GetSelectionCenter();
	if (selectionCenter)
		DrawCursor(*selectionCenter, 0.06f);
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

	DrawActionCombo();
	ImGui::Separator();

	int selectedPoints = 0;
	int selectedCount = 0;
	int selectedCurves = 0;
	int selectedSurfaces = 0;
	std::weak_ptr<Curve> selectedCurve;
	std::weak_ptr<Surface> selectedSurface;

	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected)
		{
			selectedCount++;
			if (obj->type == ObjectType::Point)
				selectedPoints++;
			else if (obj->IsA(ObjectType::Curve))
			{
				selectedCurves++;
				auto sharedCurve = std::static_pointer_cast<Curve>(obj);
				selectedCurve = sharedCurve;
				sharedCurve->CleanExpiredPoints();
			}
			else if (obj->IsA(ObjectType::Surface))
			{
				selectedSurfaces++;
				selectedSurface = std::static_pointer_cast<Surface>(obj);
			}
		}
	}

	if (selectedCurves != 1)
		selectedCurve.reset();

	if (m_menuState == MenuState::List)
	{
		Curve* activeCurve = nullptr;
		if (auto curve = selectedCurve.lock())
			activeCurve = curve.get();
		DrawListMenu(selectedCount, selectedPoints, activeCurve);

		if (auto curve = selectedCurve.lock())
		{
			if (curve->selected)
				DrawCurveList(curve.get(), selectedCount);
		}

		if (auto surface = selectedSurface.lock())
		{
			if (surface->selected && selectedSurfaces == 1)
				DrawSurfaceList(surface.get(), selectedCount);
		}
	}
	else if (m_menuState == MenuState::Edit)
		DrawEditMenu();
	else if (m_menuState == MenuState::EditGroup)
		DrawEditGroupMenu(selectedCount);


	DrawCameraSettingsMenu();
	DrawCursorSettingsMenu();

	ImGui::End();
	ImGui::Render();
}

void CadApplication::DrawListMenu(int selectedCount, int selectedPoints, Curve* activeCurve)
{

	if (ImGui::Button("Add Torus"))
	{
		m_sceneObjects.push_back(std::make_shared<Torus>(m_cursorPosition));
	}

	if (ImGui::Button("Add Point"))
	{
		auto newPoint = std::make_shared<Point>(m_cursorPosition);
		m_sceneObjects.push_back(newPoint);
		if (activeCurve)
			activeCurve->m_controlPoints.push_back(newPoint);
	}

	ImGui::BeginDisabled(selectedPoints == 0);
	if (ImGui::Button("Add Bezier Curve"))
	{
		std::vector<std::weak_ptr<Point>> pts;
		for (const auto& obj : m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
				pts.push_back(std::static_pointer_cast<Point>(obj));
		}
		m_sceneObjects.push_back(std::make_shared<BezierCurve>(std::move(pts)));
	}
	ImGui::EndDisabled();

	std::vector<std::shared_ptr<Point>> pointsToAdd;
	if (activeCurve)
	{
		for (const auto& obj : m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
			{
				auto pt = std::static_pointer_cast<Point>(obj);
				bool exists = false;

				for (const auto& cpWeak : activeCurve->m_controlPoints)
				{
					if (cpWeak.lock() == pt)
					{
						exists = true;
						break;
					}
				}

				if (!exists)
					pointsToAdd.push_back(pt);
			}
		}
	}

	ImGui::BeginDisabled(selectedPoints == 0);
	if (ImGui::Button("Add B-Spline (C2) Curve"))
	{
		std::vector<std::weak_ptr<Point>> pts;
		for (const auto& obj : m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
				pts.push_back(std::static_pointer_cast<Point>(obj));
		}
		m_sceneObjects.push_back(std::make_shared<BSplineCurve>(std::move(pts)));
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(selectedPoints == 0);
	if (ImGui::Button("Add Interpolating Spline (C2)"))
	{
		std::vector<std::weak_ptr<Point>> pts;
		for (const auto& obj : m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
				pts.push_back(std::static_pointer_cast<Point>(obj));
		}
		m_sceneObjects.push_back(std::make_shared<InterpolatingCurve>(std::move(pts)));
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(pointsToAdd.empty());
	if (ImGui::Button("Add Points to Curve"))
	{
		activeCurve->m_controlPoints.insert(activeCurve->m_controlPoints.end(), pointsToAdd.begin(), pointsToAdd.end());
	}
	ImGui::EndDisabled();

	if (ImGui::Button("Create C0 Surface"))
	{
		m_previewType = 0;
		m_previewSurface = nullptr;
		m_showSurfacePopup = true;
	}
	if (ImGui::Button("Create C2 Surface"))
	{
		m_previewType = 1;
		m_previewSurface = nullptr;
		m_showSurfacePopup = true;
	}

	if (m_showSurfacePopup)
	{
		const char* popupTitle = (m_previewType == 0) ? "C0 Surface Parameters" : "C2 Surface Parameters";
		ImGui::Begin(popupTitle, &m_showSurfacePopup);

		bool changed = (m_previewSurface == nullptr);
		changed |= ImGui::RadioButton("Flat", &m_previewShape, 0);
		ImGui::SameLine();
		changed |= ImGui::RadioButton("Cylinder", &m_previewShape, 1);

		constexpr int MAX_SEGMENTS = 50;
		int minSegU = (m_previewShape == 1) ? 3 : 1;

		int oldU = m_previewSegU;
		int oldV = m_previewSegV;

		changed |= ImGui::SliderInt("Segments U", &m_previewSegU, minSegU, 10);
		changed |= ImGui::SliderInt("Segments V", &m_previewSegV, 1, 10);

		m_previewSegU = std::clamp(m_previewSegU, minSegU, MAX_SEGMENTS);
		m_previewSegV = std::clamp(m_previewSegV, 1, MAX_SEGMENTS);
		changed |= (m_previewSegU != oldU || m_previewSegV != oldV);

		if (m_previewShape == 0)
		{
			changed |= ImGui::DragFloat("Width", &m_previewWidth, 0.1f, 0.1f, 100.0f);
			changed |= ImGui::DragFloat("Length", &m_previewDim2, 0.1f, 0.1f, 100.0f);
		}
		else
		{
			changed |= ImGui::DragFloat("Radius", &m_previewRadius, 0.1f, 0.1f, 100.0f);
			changed |= ImGui::DragFloat("Height", &m_previewDim2, 0.1f, 0.1f, 100.0f);
		}

		if (changed)
		{
			auto [surface, points] = GenerateSurface();
			m_previewSurface = std::move(surface);
			m_previewPoints = std::move(points);
		}

		if (ImGui::Button("Add to Scene"))
		{
			// 1. Rename and Add Surface
			m_previewSurface->Commit();
			std::shared_ptr<Surface> sharedSurface = std::move(m_previewSurface);

			// 2. Add all its points to the scene so they render and can be edited
			for (auto& pt : m_previewPoints)
			{
				pt->Commit();
				pt->AddDependent(sharedSurface);
				m_sceneObjects.push_back(pt);
			}
			m_previewPoints.clear();

			m_sceneObjects.push_back(sharedSurface);
			m_previewSurface = nullptr;
			m_showSurfacePopup = false;
		}
		ImGui::End();
	}

	ImGui::Separator();

	ImGui::InputText("##Search", m_filterBuffer, IM_ARRAYSIZE(m_filterBuffer));
	ImGui::SameLine();
	if (ImGui::Button("Clear"))
		m_filterBuffer[0] = '\0';
	std::string filterStr(m_filterBuffer);

	const ImGuiIO& io = ImGui::GetIO();
	ImGui::BeginChild("##ObjectListRegion", ImVec2(0, 250), true);
	for (int i = 0; i < m_sceneObjects.size(); i++)
	{
		auto& obj = m_sceneObjects[i];
		if (!filterStr.empty() && !ContainsCaseInsensitive(obj->name, filterStr))
			continue;
		ImGui::PushID(i);

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
	ImGui::EndChild();

	ImGui::Separator();

	ImGui::BeginDisabled(selectedCount == 0);
	if (selectedCount == 1)
	{
		if (ImGui::Button("Edit Selected", ImVec2(-1, 0)))
			m_menuState = MenuState::Edit;
	}
	else
	{
		if (ImGui::Button("Edit Group", ImVec2(-1, 0)))
			m_menuState = MenuState::EditGroup;
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

	if (ImGui::Checkbox("Show Bernstein", &m_showBernsteinPoints))
	{
		m_enableVirtualEdit = false;
		m_activeVirtualEdit = std::nullopt;
		m_isEditing = false;
	}

	ImGui::BeginDisabled(!m_showBernsteinPoints);
	if (ImGui::Checkbox("Enable Virtual Edit", &m_enableVirtualEdit))
	{
		m_activeVirtualEdit = std::nullopt;
		m_isEditing = false;
	}
	ImGui::EndDisabled();
}

void CadApplication::DrawCurveList(Curve* curve, int selectedCount)
{
	ImGui::TextDisabled("Selected Curve Control Points:");
	ImGui::Text("%s", curve->name.c_str());

	if (ImGui::Button("Select All Points", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = true;
		}
		m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All Points", ImVec2(-1, 0)))
	{
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = false;
		}
		m_selectionDirty = true;
	}

	const ImGuiIO& io = ImGui::GetIO();
	if (ImGui::BeginListBox(("##PointsList_" + curve->name).c_str(), ImVec2(-1.0f, 0.0f)))
	{
		for (size_t i = 0; i < curve->m_controlPoints.size(); i++)
		{
			if (auto cp = curve->m_controlPoints[i].lock())
			{
				ImGui::PushID(static_cast<int>(i));
				std::string label = std::to_string(i) + ": " + cp->name;

				if (ImGui::Selectable(label.c_str(), cp->selected))
				{
					HandleCurveListSelection(curve, i, io.KeyCtrl, io.KeyShift);
				}
				ImGui::PopID();
			}
		}
		ImGui::EndListBox();
	}

	bool pointSelected = false;
	for (const auto& cpWeak : curve->m_controlPoints)
	{
		if (auto cp = cpWeak.lock())
		{
			if (cp->selected)
			{
				pointSelected = true;
				break;
			}
		}
	}
	ImGui::BeginDisabled(!pointSelected);
	if (ImGui::Button("Remove Selected Points from Curve", ImVec2(-1, 0)))
	{
		std::erase_if(curve->m_controlPoints, [](const auto& cpWeak) {
			if (auto cp = cpWeak.lock())
			{
				if (cp->selected)
				{
					cp->selected = false;
					return true;
				}
				return false;
			}
			return true;
			});
		m_selectionDirty = true;
	}
	ImGui::EndDisabled();
	ImGui::Separator();

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Curve Only", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		std::erase_if(m_sceneObjects, [curve](const auto& obj) {
			return obj.get() == curve;
			});
	}

	ImGui::SameLine();
	if (ImGui::Button("Delete Curve and Control Points", ImVec2(-1, 0)))
	{
		std::vector<Point*> pointsToDelete;
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				pointsToDelete.push_back(cp.get());
		}

		std::erase_if(m_sceneObjects, [curve, &pointsToDelete](const auto& obj) {
			if (obj.get() == curve)
				return true;
			return std::find(pointsToDelete.begin(), pointsToDelete.end(), obj.get()) != pointsToDelete.end();
			});
	}
	ImGui::PopStyleColor(2);
}

void CadApplication::DrawSurfaceList(Surface* surface, int selectedCount)
{
	ImGui::TextDisabled("Selected Surface Control Points:");
	ImGui::Text("%s", surface->name.c_str());

	if (ImGui::Button("Select All Points", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = true;
		}
		m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All Points", ImVec2(-1, 0)))
	{
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = false;
		}
		m_selectionDirty = true;
	}

	const ImGuiIO& io = ImGui::GetIO();
	if (ImGui::BeginListBox(("##SurfacePointsList_" + surface->name).c_str(), ImVec2(-1.0f, 0.0f)))
	{
		int pointsU = (surface->shapeType == SurfaceShape::Cylinder) ? (3 * surface->segmentsU) : (3 * surface->segmentsU + 1);

		for (size_t i = 0; i < surface->m_controlPoints.size(); i++)
		{
			if (auto cp = surface->m_controlPoints[i].lock())
			{
				ImGui::PushID(static_cast<int>(i));

				int u = i % pointsU;
				int v = i / pointsU;
				std::string label = "[" + std::to_string(u) + ", " + std::to_string(v) + "] " + cp->name;

				if (ImGui::Selectable(label.c_str(), cp->selected))
				{
					HandleSurfaceListSelection(surface, i, io.KeyCtrl, io.KeyShift);
				}
				ImGui::PopID();
			}
		}
		ImGui::EndListBox();
	}

	ImGui::Separator();

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Surface Only", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		// Unlock the points so the user can delete them later
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->isLockedToSurface = false;
		}

		// Delete just the surface object
		std::erase_if(m_sceneObjects, [surface](const auto& obj) {
			return obj.get() == surface;
			});

		ClearSelection();
	}

	ImGui::SameLine();
	if (ImGui::Button("Delete Surface and Control Points", ImVec2(-1, 0)))
	{
		std::vector<Point*> pointsToDelete;
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				pointsToDelete.push_back(cp.get());
		}

		// delete the surface and the points
		std::erase_if(m_sceneObjects, [surface, &pointsToDelete](const auto& obj) {
			if (obj.get() == surface)
				return true;
			return std::find(pointsToDelete.begin(), pointsToDelete.end(), obj.get()) != pointsToDelete.end();
			});

		ClearSelection();
	}
	ImGui::PopStyleColor(2);
}

void CadApplication::HandleSurfaceListSelection(Surface* surface, size_t index, bool ctrlHeld, bool shiftHeld)
{
	auto setSelection = [&](size_t i, bool state) {
		if (auto cp = surface->m_controlPoints[i].lock())
			cp->selected = state;
		};

	if (ctrlHeld && shiftHeld)
	{
		if (m_lastCurveClickedIndex.has_value())
		{
			size_t start = std::min(index, *m_lastCurveClickedIndex);
			size_t end = std::max(index, *m_lastCurveClickedIndex);
			for (size_t j = start; j <= end; j++)
				setSelection(j, true);
		}
	}
	else if (shiftHeld)
	{
		auto anchor = m_lastCurveClickedIndex;
		if (anchor.has_value())
		{
			size_t start = std::min(index, *anchor);
			size_t end = std::max(index, *anchor);
			for (size_t j = start; j <= end; j++)
				setSelection(j, true);
			m_lastCurveClickedIndex = anchor;
		}
		else
		{
			setSelection(index, true);
			m_lastCurveClickedIndex = index;
		}
	}
	else if (ctrlHeld)
	{
		if (auto cp = surface->m_controlPoints[index].lock())
			cp->selected = !cp->selected;
		m_lastCurveClickedIndex = index;
	}
	else
	{
		for (size_t i = 0; i < surface->m_controlPoints.size(); i++)
			setSelection(i, i == index);
		m_lastCurveClickedIndex = index;
	}

	m_selectionDirty = true;
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

		if (auto point = selectedObj->As<Point>())
			DrawPointMenu(*point);
		else if (auto torus = selectedObj->As<Torus>())
			DrawTorusMenu(*torus);
		else if (auto surface = selectedObj->As<Surface>())
			DrawSurfaceMenu(*surface);
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

	Vec3f eulerDegrees = torus.m_eulerAngles.ToVec3f() * (180.0f / std::numbers::pi_v<float>);
	if (ImGui::DragFloat3("Rotation", eulerDegrees.f, 1.0f, 0.0f, 0.0f, "%.2f"))
	{
		eulerDegrees *= (std::numbers::pi_v<float> / 180.0f);
		torus.m_eulerAngles = eulerDegrees;
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
}

void CadApplication::DrawPointMenu(Point& selectedObj)
{
	if (ImGui::DragFloat3("Position", &selectedObj.m_position.x, 0.01f))
		m_selectionDirty = true;
}

void CadApplication::DrawSurfaceMenu(Surface& surface)
{
	ImGui::Text("Tessellation Parameters");
	ImGui::Spacing();

	// Sliders to control the shader density
	ImGui::SliderInt("Lines per Segment (U)", &surface.m_linesPerSegmentU, 2, 64);
	ImGui::SliderInt("Lines per Segment (V)", &surface.m_linesPerSegmentV, 2, 64);
	ImGui::SliderInt("Smoothness", &surface.m_smoothness, 2, 64);

	ImGui::Separator();
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

	ImGui::Separator();
	ImGui::Text("Stereoscopy (Anaglyph 3D)");
	ImGui::Checkbox("Enable Stereoscopy", &m_enableStereo);

	ImGui::BeginDisabled(!m_enableStereo);
	ImGui::SliderFloat("Eye Separation", &m_eyeSeparation, 0.01f, 1.0f);
	ImGui::SliderFloat("Focal Distance", &m_focalLength, 0.1f, 100.0f);
	ImGui::ColorEdit3("Left Eye Tint", &m_leftEyeColor.x);
	ImGui::ColorEdit3("Right Eye Tint", &m_rightEyeColor.x);
	ImGui::EndDisabled();

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
