#include "pch.h"
#include "CadApplication.h"
#include "../MathLib/Mat4f.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"
#include "HoleDetection.h"
#include "GeometryMath.h"
#include "SceneSerializer.h"


using namespace MathLib;
using namespace GeometryMath;
using namespace std;
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace
{
	unsigned int ExtractIndexFromName(const std::string& name)
	{
		size_t lastDigitIdx = name.find_last_of("0123456789");
		if (lastDigitIdx == std::string::npos)
			return 0;

		size_t firstDigitIdx = lastDigitIdx;
		while (firstDigitIdx > 0 && std::isdigit(static_cast<unsigned char>(name[firstDigitIdx - 1])))
			firstDigitIdx--;

		std::string numStr = name.substr(firstDigitIdx, (lastDigitIdx - firstDigitIdx) + 1);
		try
		{
			return static_cast<unsigned int>(std::stoi(numStr));
		}
		catch (...)
		{
			return 0;
		}
	}
}

CadApplication::CadApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	:DxApplication(hInstance, wndWidth, wndHeight, wndTitle), m_cadGuiManager(*this), m_camGuiManager(*this), m_sceneRenderer(m_device)
{
	SIZE wndSize = m_window.getClientSize();
	m_depthBuffer = m_device.CreateDepthStencilView(wndSize);
	auto backBuffer = m_backBuffer.Get();
	m_device.getContext()->OMSetRenderTargets(1, &backBuffer, m_depthBuffer.Get());
	Viewport viewport{ wndSize };
	m_device.getContext()->RSSetViewports(1, &viewport);

	//const auto vsByteCode = DxDevice::LoadByteCode(L"VertexShader.cso");
	//const auto psByteCode = DxDevice::LoadByteCode(L"PixelShader.cso");
	//const auto pointVsByteCode = DxDevice::LoadByteCode(L"PointVS.cso");
	//const auto pointPsByteCode = DxDevice::LoadByteCode(L"PointPS.cso");
	//const auto pointGsByteCode = DxDevice::LoadByteCode(L"PointGS.cso");
	//const auto bezierVsByteCode = DxDevice::LoadByteCode(L"BezierVS.cso");
	//const auto bezierGsByteCode = DxDevice::LoadByteCode(L"BezierGS.cso");
	//const auto surfaceVsByteCode = DxDevice::LoadByteCode(L"SurfaceVS.cso");
	//const auto surfaceHsByteCode = DxDevice::LoadByteCode(L"SurfaceHS.cso");
	//const auto surfaceDsByteCode = DxDevice::LoadByteCode(L"SurfaceDS.cso");
	//const auto gregoryVsByteCode = DxDevice::LoadByteCode(L"GregoryVS.cso");
	//const auto gregoryHsByteCode = DxDevice::LoadByteCode(L"GregoryHS.cso");
	//const auto gregoryDsByteCode = DxDevice::LoadByteCode(L"GregoryDS.cso");
	//const auto torusVSByteCode = DxDevice::LoadByteCode(L"TorusVS.cso");
	//const auto trimPSByteCode = DxDevice::LoadByteCode(L"TrimPS.cso");

	//m_vertexShader = m_device.CreateVertexShader(vsByteCode);
	//m_pixelShader = m_device.CreatePixelShader(psByteCode);
	//m_pointVertexShader = m_device.CreateVertexShader(pointVsByteCode);
	//m_pointPixelShader = m_device.CreatePixelShader(pointPsByteCode);
	//m_pointGeometryShader = m_device.CreateGeometryShader(pointGsByteCode);
	//m_bezierVertexShader = m_device.CreateVertexShader(bezierVsByteCode);
	//m_bezierGeometryShader = m_device.CreateGeometryShader(bezierGsByteCode);
	//m_surfaceVertexShader = m_device.CreateVertexShader(surfaceVsByteCode);
	//m_surfaceDomainShader = m_device.CreateDomainShader(surfaceDsByteCode);
	//m_surfaceHullShader = m_device.CreateHullShader(surfaceHsByteCode);
	//m_gregoryVertexShader = m_device.CreateVertexShader(gregoryVsByteCode);
	//m_gregoryDomainShader = m_device.CreateDomainShader(gregoryDsByteCode);
	//m_gregoryHullShader = m_device.CreateHullShader(gregoryHsByteCode);
	//m_torusVertexShader = m_device.CreateVertexShader(torusVSByteCode);
	//m_trimPixelShader = m_device.CreatePixelShader(trimPSByteCode);

	//vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
	//	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	//};
	//m_layout = m_device.CreateInputLayout(inputElements, vsByteCode);

	//std::vector<D3D11_INPUT_ELEMENT_DESC> uvInputElements = {
	//	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	//	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	//};
	//m_layoutUV = m_device.CreateInputLayout(uvInputElements, torusVSByteCode);


	//// Create the constant buffers for MVP matrices
	//m_cbPerPass = m_device.CreateConstantBuffer<PerPassBuffer>();
	//m_cbPerObject = m_device.CreateConstantBuffer<PerObjectBuffer>();
	//InitStereoBlendStates();
	InitImGui();

	//D3D11_TEXTURE2D_DESC texDesc = {};
	//texDesc.Width = 1;
	//texDesc.Height = 1;
	//texDesc.MipLevels = 1;
	//texDesc.ArraySize = 1;
	//texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	//texDesc.SampleDesc.Count = 1;
	//texDesc.Usage = D3D11_USAGE_IMMUTABLE;
	//texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

	//uint32_t whitePixel = 0xFFFFFFFF;
	//D3D11_SUBRESOURCE_DATA initData = { &whitePixel, sizeof(uint32_t), 0 };
	//Microsoft::WRL::ComPtr<ID3D11Texture2D> dummyTex;
	//m_device.get()->CreateTexture2D(&texDesc, &initData, dummyTex.GetAddressOf());
	//m_device.get()->CreateShaderResourceView(dummyTex.Get(), nullptr, m_dummyTrimTextureSRV.GetAddressOf());

	//D3D11_SAMPLER_DESC sampDesc = {};
	//sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	//sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
	//sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
	//sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
	//sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	//m_device.get()->CreateSamplerState(&sampDesc, m_trimSampler.GetAddressOf());


	//Cursor3D::InitSharedGeometry(m_device);
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
		pt->NotifyDependents();
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
	m_camera.UpdateProjectionMatrix();
	//if (m_camera.UpdateProjectionMatrix())
	//	SyncPerPassBuffer();
}

void CadApplication::UpdateViewMatrix()
{
	m_camera.UpdateViewMatrices();
	//if (m_camera.UpdateViewMatrices())
	//	SyncPerPassBuffer();
}

//void CadApplication::SyncPerPassBuffer()
//{
//	if (m_cbPerPass)
//	{
//		PerPassBuffer perPassData;
//		perPassData.viewProj = m_camera.GetProjViewMatrix();
//		perPassData.aspectRatio = m_camera.GetAspectRatio();
//		perPassData.renderSize[0] = static_cast<float>(m_renderSize.cx);
//		perPassData.renderSize[1] = static_cast<float>(m_renderSize.cy);
//		perPassData.stereoTint = { 1.0f, 1.0f, 1.0f, 1.0f };
//		m_device.UpdateBuffer(m_cbPerPass, perPassData);
//	}
//}

void CadApplication::ClearSelection()
{
	for (auto& o : m_sceneObjects)
		o->selected = false;

	ResetSelectionState();
}

void CadApplication::ResetSelectionState()
{
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

//void CadApplication::DrawCursor(float3 position, float scale)
//{
//	auto& context = m_device.getContext();
//
//	PerObjectBuffer objData;
//	Mat4f translation = Mat4f::Translation(position.x, position.y, position.z);
//	Mat4f scaling = Mat4f::Scaling(scale);
//
//	// X axis - Red
//	objData.model = translation * scaling;
//	objData.color = { 1.0f, 0.0f, 0.0f, 1.0f };
//	m_device.UpdateBuffer(m_cbPerObject, objData);
//	context->Draw(Cursor3D::VertexCount, 0);
//
//	// Y axis - Green (Rotate X arrow 90 degrees around Z axis)
//	objData.model = translation * Mat4f::RotationZ(std::numbers::pi_v<float> / 2.0f) * scaling;
//	objData.color = { 0.0f, 1.0f, 0.0f, 1.0f };
//	m_device.UpdateBuffer(m_cbPerObject, objData);
//	context->Draw(Cursor3D::VertexCount, 0);
//
//	// Z axis - Blue (Rotate X arrow -90 degrees around Y axis)
//	objData.model = translation * Mat4f::RotationY(-std::numbers::pi_v<float> / 2.0f) * scaling;
//	objData.color = { 0.0f, 0.0f, 1.0f, 1.0f };
//	m_device.UpdateBuffer(m_cbPerObject, objData);
//	context->Draw(Cursor3D::VertexCount, 0);
//}

void CadApplication::AddTorusAtCursor()
{
	m_sceneObjects.push_back(std::make_shared<Torus>(m_cursorPosition));
}

void CadApplication::AddPointAtCursor(std::shared_ptr<Curve> targetCurve)
{
	auto newPoint = std::make_shared<Point>(m_cursorPosition);
	m_sceneObjects.push_back(newPoint);
	if (targetCurve)
	{
		targetCurve->m_controlPoints.push_back(newPoint);
		newPoint->AddDependent(targetCurve);
		targetCurve->MarkDirty();
	}
}

std::shared_ptr<Curve> CadApplication::GetSingleSelectedCurve() const
{
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
	return selectedCurves == 1 ? activeCurve : nullptr;
}

void CadApplication::AddSurface()
{
	auto result = m_surfaceBuilder.Build(m_device);
	std::shared_ptr<Surface> sharedSurface = std::move(result.surface);
	for (auto& pt : result.points)
	{
		pt->AddDependent(sharedSurface);
		m_sceneObjects.push_back(pt);
	}

	m_sceneObjects.push_back(sharedSurface);
}

void CadApplication::DeleteSelectedObjects()
{
	auto unbindTexture = [](std::weak_ptr<SceneObject> objWeak, ID3D11ShaderResourceView* texToMatch) {
		if (auto obj = objWeak.lock())
		{
			if (auto surf = obj->As<Surface>())
			{
				if (surf->m_trimTextureSRV.Get() == texToMatch)
					surf->m_trimTextureSRV.Reset();
			}
			else if (obj->type == ObjectType::Torus)
			{
				auto torus = static_cast<Torus*>(obj.get());
				if (torus->m_trimTextureSRV.Get() == texToMatch)
					torus->m_trimTextureSRV.Reset();
			}
		}
		};

	for (auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;

		if (auto surface = obj->As<Surface>())
		{
			for (const auto& cpWeak : surface->m_controlPoints)
			{
				if (auto cp = cpWeak.lock())
					cp->m_surfaceLockCount = std::max(0, cp->m_surfaceLockCount - 1);
			}
		}
		else if (auto patch = obj->As<GregoryPatch>())
		{
			for (int i = 0; i < 3; i++)
			{
				const auto& edge = patch->m_hole.edges[i];
				for (const auto& cpWeak : edge.boundaryPoints)
				{
					if (auto cp = cpWeak.lock())
						cp->m_surfaceLockCount = std::max(0, cp->m_surfaceLockCount - 1);
				}
				for (const auto& cpWeak : edge.innerPoints)
				{
					if (auto cp = cpWeak.lock())
						cp->m_surfaceLockCount = std::max(0, cp->m_surfaceLockCount - 1);
				}
			}
		}
		else if (auto intersection = obj->As<Intersection>())
		{
			unbindTexture(intersection->m_surface1, intersection->m_trimTexture1.srv.Get());
			unbindTexture(intersection->m_surface2, intersection->m_trimTexture2.srv.Get());
		}
	}

	erase_if(m_sceneObjects, [](const auto& obj) {
		if (!obj->selected)
			return false;

		if (auto pt = obj->As<Point>())
			return pt->m_surfaceLockCount == 0;

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
	case WM_KEYDOWN:
	{
		bool isRepeat = (msg.lParam & 0x40000000) != 0;
		if (isRepeat)
			break;

		bool ctrlHeld = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
		bool shiftHeld = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;

		if (msg.wParam == 'M')
		{
			ActionMergeSelectedPoints();
			return true;
		}
		else if (msg.wParam == 'G')
		{
			ActionSealHoles();
			return true;
		}
		else if (msg.wParam == 'S')
		{
			if (ctrlHeld)
			{
				if (shiftHeld)
					ActionSaveAs();
				else
					ActionSave();
				return true;
			}
		}
		else if (msg.wParam == 'O' && ctrlHeld)
		{
			ActionLoad();
			return true;
		}
		else if (msg.wParam == 'B')
		{
			if (!m_isBoxSelecting)
			{
				POINT pt;
				GetCursorPos(&pt);
				ScreenToClient(m_window.getHandle(), &pt);

				m_isBoxSelecting = true;
				m_boxSelectStart = pt;
				m_boxSelectCurrent = pt;
				SetCapture(m_window.getHandle());
			}
			return true;
		}
		else if (msg.wParam == 'I')
		{
			m_cadGuiManager.ShowIntersectionPopup();
			return true;
		}
		break;
	}
	case WM_KEYUP:
	{
		if (msg.wParam == 'B')
		{
			if (m_isBoxSelecting)
			{
				POINT pt;
				GetCursorPos(&pt);
				ScreenToClient(m_window.getHandle(), &pt);
				m_boxSelectCurrent = pt;

				int dx = std::abs(m_boxSelectCurrent.x - m_boxSelectStart.x);
				int dy = std::abs(m_boxSelectCurrent.y - m_boxSelectStart.y);

				if (dx >= 3 || dy >= 3)
				{
					bool ctrlHeld = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
					bool shiftHeld = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
					PerformBoxSelection(ctrlHeld, shiftHeld);
				}

				m_isBoxSelecting = false;
				ReleaseCapture();
			}
			return true;
		}
		break;
	}
	case WM_LBUTTONDOWN:
	{
		if (m_appMode == AppMode::CAM_Simulation)
			return true;

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

			AddPointAtCursor(GetSingleSelectedCurve());
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
			if (m_currentEditAction != EditAction::None && !ctrlHeld)
				BeginEditAction(xPos, yPos, shiftHeld);
			else
			{
				auto [normX, normY] = CalculateCoordsFromPixel(static_cast<float>(xPos), static_cast<float>(yPos),
					static_cast<float>(m_renderSize.cx), static_cast<float>(m_renderSize.cy));
				m_cursorPosition = m_camera.GetPositionOnFocalPlane(normX, normY);

				if (!ctrlHeld)
					ClearSelection();
			}

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
		if (m_isBoxSelecting)
			m_boxSelectCurrent = { xPos, yPos };
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
	Vec3f scaleMultiplier(1.0f, 1.0f, 1.0f);
	//float scaleFactor = 1.0f;
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
	else if (m_currentEditAction >= EditAction::Scale && m_currentEditAction <= EditAction::ScaleZ)
	{
		float scale = std::max(0.01f, 1.0f + totalDx * 0.01f);
		if (m_currentEditAction == EditAction::Scale) scaleMultiplier = { scale, scale, scale };
		else if (m_currentEditAction == EditAction::ScaleX) scaleMultiplier.x = scale;
		else if (m_currentEditAction == EditAction::ScaleY) scaleMultiplier.y = scale;
		else if (m_currentEditAction == EditAction::ScaleZ) scaleMultiplier.z = scale;
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
					Vec3f newPos = editPivot + offset * scaleMultiplier;
					obj->m_position = newPos;

					if (auto torus = obj->As<Torus>())
						torus->SetScale(torus->m_baseScale.ToVec3f() * scaleMultiplier);
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
	float dt = static_cast<float>(m_clock.Query());
	dt = std::min(dt, 0.1f); // Limit delta time to avoid large jumps in case of a pause or breakpoint
	if (m_appMode == AppMode::CAM_Simulation)
		m_millingSimulator.Update(dt);

	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	if (m_appMode == AppMode::CAD_Modeling)
		m_cadGuiManager.DrawMenu();
	else
		m_camGuiManager.DrawMenu();

	ImGui::Render();

	auto& context = m_device.getContext();
	const float clear_color[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	context->ClearRenderTargetView(m_backBuffer.Get(), clear_color);
	context->OMSetRenderTargets(1, m_backBuffer.GetAddressOf(), m_depthBuffer.Get());

	m_sceneRenderer.DrawScene(
		m_camera,
		m_sceneObjects,
		m_enableStereo,
		m_leftEyeColor,
		m_rightEyeColor,
		m_eyeSeparation,
		m_focalLength,
		m_cursorPosition,
		GetSelectionCenter(),
		m_showBernsteinPoints,
		m_renderSize,
		m_cadGuiManager.IsSurfacePreviewActive(),
		m_surfaceBuilder,
		m_depthBuffer.Get()
	);

	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}


void CadApplication::ActionSave()
{
	if (m_currentFilePath.empty())
		ActionSaveAs();
	else
		SaveScene(m_currentFilePath);
}

void CadApplication::ActionSaveAs()
{
	std::wstring path = ShowFileDialog<IFileSaveDialog>();
	if (!path.empty())
	{
		m_currentFilePath = path;
		SaveScene(m_currentFilePath);
	}
}

void CadApplication::SaveScene(const std::wstring& filePath)
{
	try
	{
		SceneSerializer::Save(filePath, m_sceneObjects);
	}
	catch (const std::exception& e)
	{
		MessageBoxA(m_window.getHandle(), e.what(), "Save Error", MB_OK | MB_ICONERROR);
	}
}

void CadApplication::ActionLoad()
{
	std::wstring path = ShowFileDialog<IFileOpenDialog>();
	if (!path.empty())
		LoadScene(path);
}

void CadApplication::LoadScene(const std::wstring& filePath)
{
	try
	{
		auto loadedObjects = SceneSerializer::Load(filePath, m_device);
		ResetSelectionState();
		m_sceneObjects = std::move(loadedObjects);
		m_currentFilePath = filePath;
	}
	catch (const std::exception& e)
	{
		MessageBoxA(m_window.getHandle(), e.what(), "Error", MB_OK | MB_ICONERROR);
	}
}

void CadApplication::ActionMergeSelectedPoints()
{
	std::vector<std::shared_ptr<Point>> pointsToMerge;
	MathLib::Vec3f sumPos(0, 0, 0);

	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected && obj->type == ObjectType::Point)
		{
			auto pt = std::static_pointer_cast<Point>(obj);
			pointsToMerge.push_back(pt);
			sumPos += pt->m_position.ToVec3f();
		}
	}
	if (pointsToMerge.size() < 2)
		return;

	MathLib::Vec3f avgPos = sumPos / pointsToMerge.size();
	auto mergedPoint = std::make_shared<Point>(avgPos);
	m_sceneObjects.push_back(mergedPoint);

	//for (auto& oldPt : pointsToMerge)
	//{
	//	for (auto& weakDep : oldPt->m_dependents)
	//	{
	//		if (auto dep = weakDep.lock())
	//		{
	//			dep->ReplacePoint(oldPt.get(), mergedPoint);
	//			mergedPoint->AddDependent(dep);
	//		}
	//	}
	//}

	std::vector<std::shared_ptr<IPointDependent>> uniqueDependents;
	for (auto& oldPt : pointsToMerge)
	{
		for (auto& weakDep : oldPt->m_dependents)
		{
			if (auto dep = weakDep.lock())
			{
				dep->ReplacePoint(oldPt.get(), mergedPoint);
				if (std::find(uniqueDependents.begin(), uniqueDependents.end(), dep) == uniqueDependents.end())
					uniqueDependents.push_back(dep);
			}
		}
	}

	mergedPoint->AddDependents(uniqueDependents);

	ClearSelection();
	mergedPoint->selected = true;
	std::erase_if(m_sceneObjects, [&](const std::shared_ptr<SceneObject>& obj) {
		return obj->type == ObjectType::Point && std::find(pointsToMerge.begin(), pointsToMerge.end(), obj) != pointsToMerge.end();
		});
	m_selectionDirty = true;
}

void CadApplication::ActionSealHoles()
{
	std::vector<std::shared_ptr<BezierSurface>> selectedSurfaces;
	for (const auto& obj : m_sceneObjects)
	{
		if (obj->selected && obj->type == ObjectType::BezierSurface)
			selectedSurfaces.push_back(std::static_pointer_cast<BezierSurface>(obj));
	}

	std::vector<Hole3Cycle> holes = Find3SidedHoles(selectedSurfaces);
	for (auto& hole : holes)
	{
		auto patch = std::make_shared<GregoryPatch>(std::move(hole));
		for (int i = 0; i < 3; i++)
		{
			auto& edge = patch->m_hole.edges[i];
			for (auto& cpWeak : edge.boundaryPoints)
			{
				if (auto cpShared = cpWeak.lock())
				{
					cpShared->AddDependent(patch);
					cpShared->m_surfaceLockCount++;
				}
			}
			for (auto& cpWeak : edge.innerPoints)
			{
				if (auto cpShared = cpWeak.lock())
				{
					cpShared->AddDependent(patch);
					cpShared->m_surfaceLockCount++;
				}
			}
		}
		patch->InitGeometry(m_device);
		m_sceneObjects.push_back(patch);
	}
}

void CadApplication::PerformBoxSelection(bool ctrlHeld, bool shiftHeld)
{
	auto [minX, maxX] = std::minmax(m_boxSelectStart.x, m_boxSelectCurrent.x);
	auto [minY, maxY] = std::minmax(m_boxSelectStart.y, m_boxSelectCurrent.y);

	Mat4f projView = m_camera.GetProjViewMatrix();

	if (!ctrlHeld && !shiftHeld)
		ClearSelection();

	bool select = !ctrlHeld;

	for (const auto& obj : m_sceneObjects)
	{
		if (auto point = obj->As<Point>())
		{
			Vec4f worldPos = point->m_position.ToVec4f(1.0f);
			Vec4f clipPos = projView * worldPos;

			if (clipPos.w <= 0.0f)
				continue;

			clipPos /= clipPos.w;
			float screenX = (clipPos.x + 1.0f) * 0.5f * m_renderSize.cx;
			float screenY = (1.0f - clipPos.y) * 0.5f * m_renderSize.cy;

			if (screenX >= minX && screenX <= maxX && screenY >= minY && screenY <= maxY)
				point->selected = select;
		}
	}

	m_selectionDirty = true;
}

void CadApplication::ActionFindIntersection()
{
	std::vector<std::weak_ptr<SceneObject>> selectedObjects;
	for (const auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;
		if (obj->type == ObjectType::Torus || obj->IsA(ObjectType::Surface))
			selectedObjects.push_back(obj);
	}

	if (selectedObjects.size() != 1 && selectedObjects.size() != 2)
		return;

	bool isSelfIntersection = (selectedObjects.size() == 1);

	auto& obj1W = selectedObjects[0];
	auto& obj2W = isSelfIntersection ? selectedObjects[0] : selectedObjects[1];

	auto obj1 = obj1W.lock().get();
	auto obj2 = obj2W.lock().get();

	if (isSelfIntersection && !obj1->IsA(ObjectType::Surface))
		return;

	Vec4f maxDomains{ 1.0f, 1.0f, 1.0f, 1.0f };
	if (auto surf1 = obj1->As<Surface>())
	{
		maxDomains.x = static_cast<float>(surf1->GetSegmentsU());
		maxDomains.y = static_cast<float>(surf1->GetSegmentsV());
	}
	if (auto surf2 = obj2->As<Surface>())
	{
		maxDomains.z = static_cast<float>(surf2->GetSegmentsU());
		maxDomains.w = static_cast<float>(surf2->GetSegmentsV());
	}

	std::optional<float3> cursorHint = std::nullopt;
	if (m_cadGuiManager.UseCursorAsHint())
		cursorHint = m_cursorPosition;

	auto resultOpt = GeometryMath::CalculateIntersection(obj1, obj2, isSelfIntersection, maxDomains, m_cadGuiManager.GetIntersectionStep(), cursorHint);

	if (!resultOpt)
		return;

	auto& result = *resultOpt;

	auto isS1Dim = [](int dim) { return dim == 0 || dim == 1; };
	auto isS2Dim = [](int dim) { return dim == 2 || dim == 3; };

	TrimFillMode trimModeS1 = TrimFillMode::None;
	TrimFillMode trimModeS2 = TrimFillMode::None;

	if (result.isClosedLoop)
	{
		trimModeS1 = TrimFillMode::ClosedLoop;
		trimModeS2 = TrimFillMode::ClosedLoop;
	}
	else
	{
		if (isS1Dim(result.forwardHitDim) && isS1Dim(result.backwardHitDim))
			trimModeS1 = TrimFillMode::BoundaryToBoundary;

		if (isS2Dim(result.forwardHitDim) && isS2Dim(result.backwardHitDim))
			trimModeS2 = TrimFillMode::BoundaryToBoundary;
	}

	auto intersection = std::make_shared<LinearIntersection>(
		std::move(result.points),
		std::move(result.params),
		obj1W, obj2W,
		std::move(maxDomains),
		trimModeS1, trimModeS2,
		result.isClosedLoop
	);

	intersection->InitGeometry(m_device);
	RenderTrimTextures(intersection.get());

	auto bindTrimTex = [&](SceneObject* obj, const Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>& srv) {
		if (auto surf = obj->As<Surface>())
			surf->m_trimTextureSRV = srv;
		else if (obj->type == ObjectType::Torus)
		{
			auto torus = static_cast<Torus*>(obj);
			torus->m_trimTextureSRV = srv;
		}
		};

	bindTrimTex(obj1, intersection->m_trimTexture1.srv);
	bindTrimTex(obj2, intersection->m_trimTexture2.srv);

	m_sceneObjects.push_back(std::move(intersection));
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