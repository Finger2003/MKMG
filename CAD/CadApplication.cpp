#include "pch.h"
#include "CadApplication.h"
#include "../MathLib/Mat4f.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"
#include "HoleDetection.h"


using namespace MathLib;
using namespace std;
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace
{
	MathLib::Vec3f ScreenToObjectArcballVector(int mouseX, int mouseY, float objScreenX, float objScreenY, float radius)
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

	pair<float, float> CalculateCoordsFromPixel(float x, float y, float width, float height)
	{
		float normX = (x / width) * 2.0f - 1.0f;
		float normY = 1.0f - (y / height) * 2.0f; // Invert Y coordinate
		return { normX, normY };
	}

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

	struct SurfaceEvalResult
	{
		Vec3f p;
		Vec3f du;
		Vec3f dv;
	};

	struct CurveEvalResult
	{
		Vec3f p;
		Vec3f d;
	};

	CurveEvalResult EvaluateCubicDeCasteljau(float t, MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3)
	{
		float u = 1.0f - t;

		// Level 1
		Vec3f p01 = p0 * u + p1 * t;
		Vec3f p12 = p1 * u + p2 * t;
		Vec3f p23 = p2 * u + p3 * t;

		// Level 2
		Vec3f p012 = p01 * u + p12 * t;
		Vec3f p123 = p12 * u + p23 * t;

		// Level 3
		Vec3f p0123 = p012 * u + p123 * t;

		// Derivative calculation
		Vec3f derivative = (p123 - p012) * 3.0f;

		return { p0123, derivative };
	}

	SurfaceEvalResult EvaluateBezierSurface(BezierSurface* surf, float u, float v)
	{
		int segU = surf->GetSegmentsU();
		int segV = surf->GetSegmentsV();

		int pu = std::clamp(static_cast<int>(u), 0, segU - 1);
		int pv = std::clamp(static_cast<int>(v), 0, segV - 1);

		float lu = u - pu;
		float lv = v - pv;


		Vec3f P[4][4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				int idx = (pv * 3 + j) * surf->m_gridPointsU + (pu * 3 + i);
				if (auto pt = surf->m_controlPoints[idx].lock())
					P[i][j] = pt->m_position.ToVec3f();
				else
					P[i][j] = MathLib::Vec3f(0.0f, 0.0f, 0.0f);
			}
		}

		Vec3f Q[4];
		Vec3f dQu[4];
		for (int j = 0; j < 4; j++)
		{
			auto [pt, deriv] = EvaluateCubicDeCasteljau(lu, P[0][j], P[1][j], P[2][j], P[3][j]);
			Q[j] = pt;
			dQu[j] = deriv;
		}

		auto resV = EvaluateCubicDeCasteljau(lv, Q[0], Q[1], Q[2], Q[3]);
		auto resDu = EvaluateCubicDeCasteljau(lv, dQu[0], dQu[1], dQu[2], dQu[3]);

		return { resV.p, resDu.p, resV.d };
	}

	CurveEvalResult EvaluateCubicDeBoor(float t, MathLib::Vec3f p0, MathLib::Vec3f p1, MathLib::Vec3f p2, MathLib::Vec3f p3)
	{
		// Level 1
		MathLib::Vec3f p1_1 = p0 * ((1.0f - t) / 3.0f) + p1 * ((t + 2.0f) / 3.0f);
		MathLib::Vec3f p2_1 = p1 * ((2.0f - t) / 3.0f) + p2 * ((t + 1.0f) / 3.0f);
		MathLib::Vec3f p3_1 = p2 * ((3.0f - t) / 3.0f) + p3 * (t / 3.0f);

		// Level 2
		MathLib::Vec3f p2_2 = p1_1 * ((1.0f - t) / 2.0f) + p2_1 * ((t + 1.0f) / 2.0f);
		MathLib::Vec3f p3_2 = p2_1 * ((2.0f - t) / 2.0f) + p3_1 * (t / 2.0f);

		// Level 3
		MathLib::Vec3f p3_3 = p2_2 * (1.0f - t) + p3_2 * t;

		// Derivative calculation
		MathLib::Vec3f derivative = (p3_2 - p2_2) * 3.0f;

		return { p3_3, derivative };
	}

	SurfaceEvalResult EvaluateBSplineSurface(BSplineSurface* surf, float u, float v)
	{
		int segU = surf->GetSegmentsU();
		int segV = surf->GetSegmentsV();

		int pu = std::clamp(static_cast<int>(u), 0, segU - 1);
		int pv = std::clamp(static_cast<int>(v), 0, segV - 1);

		float lu = u - pu;
		float lv = v - pv;

		Vec3f P[4][4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				int idx = (pv + j) * surf->m_gridPointsU + (pu + i);
				if (auto pt = surf->m_controlPoints[idx].lock())
					P[i][j] = pt->m_position.ToVec3f();
				else
					P[i][j] = Vec3f(0.0f);
			}
		}

		Vec3f Q[4];
		Vec3f dQu[4];
		for (int j = 0; j < 4; j++)
		{
			// Evaluate De Boor in the U direction
			auto [pt, deriv] = EvaluateCubicDeBoor(lu, P[0][j], P[1][j], P[2][j], P[3][j]);
			Q[j] = pt;
			dQu[j] = deriv;
		}

		auto resV = EvaluateCubicDeBoor(lv, Q[0], Q[1], Q[2], Q[3]);
		auto resDu = EvaluateCubicDeBoor(lv, dQu[0], dQu[1], dQu[2], dQu[3]);

		return { resV.p, resDu.p, resV.d };
	}

	SurfaceEvalResult EvaluateParametric(SceneObject* obj, float u, float v)
	{
		if (obj->type == ObjectType::BezierSurface)
			return EvaluateBezierSurface(static_cast<BezierSurface*>(obj), u, v);
		else if (obj->type == ObjectType::BSplineSurface)
			return EvaluateBSplineSurface(static_cast<BSplineSurface*>(obj), u, v);
		else if (obj->type == ObjectType::Torus)
		{
			Torus* torus = static_cast<Torus*>(obj);
			constexpr float dBetaDu = 2.0f * std::numbers::pi_v<float>;
			constexpr float dAlphaDv = 2.0f * std::numbers::pi_v<float>;

			float beta = u * dBetaDu;
			float alpha = v * dAlphaDv;
			float R = torus->GetMajorRadius();
			float r = torus->GetMinorRadius();

			float cosAlpha = std::cos(alpha);
			float sinAlpha = std::sin(alpha);
			float cosBeta = std::cos(beta);
			float sinBeta = std::sin(beta);

			Vec4f pLocal(
				(R + r * cosAlpha) * cosBeta,
				r * sinAlpha,
				-(R + r * cosAlpha) * sinBeta,
				1.0f
			);

			Vec4f duLocal(
				-(R + r * cosAlpha) * sinBeta * dBetaDu,
				0.0f,
				-(R + r * cosAlpha) * cosBeta * dBetaDu,
				0.0f
			);

			Vec4f dvLocal(
				-r * sinAlpha * cosBeta * dAlphaDv,
				r * cosAlpha * dAlphaDv,
				r * sinAlpha * sinBeta * dAlphaDv,
				0.0f
			);

			Mat4f model = torus->m_modelMatrix;

			Vec4f pWorld = model * pLocal;
			Vec4f duWorld = model * duLocal;
			Vec4f dvWorld = model * dvLocal;

			return {
				Vec3f::FromVec4f(pWorld),
				Vec3f::FromVec4f(duWorld),
				Vec3f::FromVec4f(dvWorld)
			};
		}
		else
			return { Vec3f(0.0f), Vec3f(0.0f), Vec3f(0.0f) };
	}

	std::optional<MathLib::Vec4f> SolveLinearSystem(MathLib::Mat4f J, MathLib::Vec4f F)
	{
		for (int i = 0; i < 4; i++)
		{
			// Partial pivoting
			int pivot = i;
			for (int j = i + 1; j < 4; j++)
			{
				if (std::abs(J.m[j][i]) > std::abs(J.m[pivot][i]))
					pivot = j;
			}
			if (std::abs(J.m[pivot][i]) < 1e-8f)
				return std::nullopt; // singular matrix

			// Swap the current row with the pivot row
			std::swap(J.rows[i], J.rows[pivot]);
			std::swap(F.f[i], F.f[pivot]);

			// Scale the pivot row so the diagonal becomes 1
			float diag = J.m[i][i];
			float invDiag = 1.0f / diag;
			J.rows[i] *= invDiag;
			F.f[i] *= invDiag;

			// Eliminate the current variable from other rows
			for (int j = 0; j < 4; j++)
			{
				if (i != j)
				{
					float factor = J.m[j][i];
					J.rows[j] = J.rows[j] - (J.rows[i] * factor);
					F.f[j] -= factor * F.f[i];
				}
			}
		}

		return F;
	}

	std::optional<MathLib::Vec4f> FindIntersectionNextPoint(
		SceneObject* obj1,
		SceneObject* obj2,
		MathLib::Vec4f startParams,
		MathLib::Vec3f p0,
		MathLib::Vec3f t,
		float d,
		int depth = 0
	)
	{
		constexpr int maxDepth = 3;
		if (depth > maxDepth)
			return std::nullopt;

		Vec4f currentParams = startParams;
		bool converged = false;
		constexpr int maxIterations = 50;
		constexpr float F_dist_tolerance = 1e-8f;
		constexpr float planeTolerance = 1e-4f;

		for (int iter = 0; iter < maxIterations; iter++)
		{
			auto [p1, du1, dv1] = EvaluateParametric(obj1, currentParams.x, currentParams.y);
			auto [p2, du2, dv2] = EvaluateParametric(obj2, currentParams.z, currentParams.w);

			Vec3f F_dist = p1 - p2;
			float planeDist = Vec3f::dot(p1 - p0, t) - d;

			if (F_dist.length_sqr() < F_dist_tolerance && std::abs(planeDist) < planeTolerance)
			{
				converged = true;
				break;
			}

			Mat4f J(
				Vec4f(du1.x, dv1.x, -du2.x, -dv2.x),
				Vec4f(du1.y, dv1.y, -du2.y, -dv2.y),
				Vec4f(du1.z, dv1.z, -du2.z, -dv2.z),
				Vec4f(Vec3f::dot(du1, t), Vec3f::dot(dv1, t), 0.0f, 0.0f)
			);

			Vec4f F(-F_dist.x, -F_dist.y, -F_dist.z, -planeDist);

			auto deltaOpt = SolveLinearSystem(J, F);
			if (!deltaOpt)
				break;

			currentParams += *deltaOpt;
		}

		if (converged)
			return currentParams;
		else
		{
			auto halfStep1 = FindIntersectionNextPoint(obj1, obj2, startParams, p0, t, d / 2.0f, depth + 1);
			if (!halfStep1)
				return std::nullopt;

			auto [p_mid, _1, _2] = EvaluateParametric(obj1, halfStep1->x, halfStep1->y);

			auto halfStep2 = FindIntersectionNextPoint(obj1, obj2, *halfStep1, p_mid, t, d / 2.0f, depth + 1);
			return halfStep2;
		}
	}

	std::optional<MathLib::Vec4f> FindExactEdgePoint(
		SceneObject* obj1,
		SceneObject* obj2,
		MathLib::Vec4f startGuess,
		int boundaryDim,
		float boundaryValue
	)
	{
		Vec4f currentParams = startGuess;
		currentParams.f[boundaryDim] = boundaryValue;
		constexpr int maxIterations = 100;
		constexpr float F_dist_tolerance = 1e-8f;

		for (int iter = 0; iter < maxIterations; iter++)
		{
			auto [p1, du1, dv1] = EvaluateParametric(obj1, currentParams.x, currentParams.y);
			auto [p2, du2, dv2] = EvaluateParametric(obj2, currentParams.z, currentParams.w);
			Vec3f F_dist = p1 - p2;

			if (F_dist.length_sqr() < F_dist_tolerance)
				return currentParams;

			Mat4f J(
				Vec4f(du1.x, dv1.x, -du2.x, -dv2.x),
				Vec4f(du1.y, dv1.y, -du2.y, -dv2.y),
				Vec4f(du1.z, dv1.z, -du2.z, -dv2.z),
				Vec4f(0.0f, 0.0f, 0.0f, 0.0f)
			);

			J.m[3][boundaryDim] = 1.0f; // Enforce boundary condition

			Vec4f F(-F_dist.x, -F_dist.y, -F_dist.z, -(currentParams.f[boundaryDim] - boundaryValue));

			auto deltaOpt = SolveLinearSystem(J, F);
			if (!deltaOpt)
				return std::nullopt;

			currentParams += *deltaOpt;
			currentParams.f[boundaryDim] = boundaryValue;
		}

		return std::nullopt;
	}

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
	const auto gregoryHsByteCode = DxDevice::LoadByteCode(L"GregoryHS.cso");
	const auto gregoryDsByteCode = DxDevice::LoadByteCode(L"GregoryDS.cso");

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
	m_gregoryDomainShader = m_device.CreateDomainShader(gregoryDsByteCode);
	m_gregoryHullShader = m_device.CreateHullShader(gregoryHsByteCode);

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
			m_showIntersectionPopup = true;
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
			{
				activeCurve->m_controlPoints.push_back(newPoint);
				newPoint->AddDependent(activeCurve);
				activeCurve->MarkDirty();
			}

			return true;
		}

		//if (bPressed)
		//{
		//	if (!ctrlHeld)
		//		ClearSelection();

		//	m_isBoxSelecting = true;
		//	m_boxSelectStart = { xPos, yPos };
		//	m_boxSelectCurrent = { xPos, yPos };
		//	SetCapture(m_window.getHandle());
		//	return true;
		//}

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
	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, Point::GetSharedVertexBuffer().GetAddressOf(), &stride, &offset);
	for (auto& obj : m_sceneObjects)
	{
		if (auto point = obj->As<Point>())
		{
			PerObjectBuffer objData;
			objData.model = point->GetModelMatrix();
			objData.color = point->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->Draw(1, 0);
		}
	}

	if (m_showSurfacePopup)
	{
		for (auto& point : m_surfaceBuilder.rawPoints)
		{
			PerObjectBuffer objData;
			objData.model = Mat4f::Translation(point.x, point.y, point.z);
			objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);
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

	if (m_showSurfacePopup && m_surfaceBuilder.m_patchIndexCount > 0)// && m_previewSurface)
	{
		UINT stride = sizeof(VertexPosition);
		UINT offset = 0;
		context->IASetVertexBuffers(0, 1, m_surfaceBuilder.m_patchVertexBuffer.GetAddressOf(), &stride, &offset);
		context->IASetIndexBuffer(m_surfaceBuilder.m_patchIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
		PerObjectBuffer objData;
		objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
		auto updateAndDraw = [&](float xParam, float yParam) {
			objData.surfaceParams =
			{
				xParam,
				yParam,
				25.0f,
				0.0f
			};
			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->DrawIndexed(m_surfaceBuilder.m_patchIndexCount, 0, 0);
			};

		updateAndDraw(0.0f, m_surfaceBuilder.linesPerSegmentU);
		updateAndDraw(1.0f, m_surfaceBuilder.linesPerSegmentV);

		//DrawSurface(context, m_previewSurface.get(), Vec4f(0.0f, 0.5f, 1.0f, 1.0f));	
	}

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

void CadApplication::DrawGregoryPatches(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	for (auto& obj : m_sceneObjects)
	{
		if (obj->type == ObjectType::GregoryPatch)
		{
			auto gregoryPatch = static_cast<GregoryPatch*>(obj.get());
			gregoryPatch->UpdateVertices(m_device);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, gregoryPatch->m_vertexBuffer.GetAddressOf(), &stride, &offset);

			PerObjectBuffer objData;
			objData.color = gregoryPatch->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);

			auto updateAndDraw = [&](float xParam, float yParam) {
				objData.surfaceParams =
				{
					xParam,
					yParam,
					static_cast<float>(gregoryPatch->m_smoothness),
					0.0f
				};
				m_device.UpdateBuffer(m_cbPerObject, objData);
				context->Draw(gregoryPatch->m_vertexCount, 0);
				};
			updateAndDraw(0.0f, static_cast<float>(gregoryPatch->m_linesPerSegmentU));
			updateAndDraw(1.0f, static_cast<float>(gregoryPatch->m_linesPerSegmentV));
		}
	}
}


void CadApplication::DrawGregoryPatchesTangents(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.1f, 0.1f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : m_sceneObjects)
	{
		if (auto patch = obj->As<GregoryPatch>())
		{
			patch->UpdateVertices(m_device);
			if (patch->selected)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, patch->m_continuityVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(patch->m_continuityVertexCount, 0);
			}
		}
	}
}

void CadApplication::DrawIntersections(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context)
{
	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	for (const auto& obj : m_sceneObjects)
	{
		if (obj->type == ObjectType::Intersection)
		{
			auto intersection = static_cast<Intersection*>(obj.get());

			if (intersection->m_vertexCount > 0)
			{
				objData.color = intersection->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
				m_device.UpdateBuffer(m_cbPerObject, objData);

				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, intersection->m_vertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(intersection->m_vertexCount, 0);
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
	DrawIntersections(context);
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	DrawSurfacesPolylines(context);
	DrawGregoryPatchesTangents(context);

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

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_20_CONTROL_POINT_PATCHLIST);
	context->HSSetShader(m_gregoryHullShader.Get(), nullptr, 0);
	context->DSSetShader(m_gregoryDomainShader.Get(), nullptr, 0);

	DrawGregoryPatches(context);

	context->HSSetShader(nullptr, nullptr, 0);
	context->DSSetShader(nullptr, nullptr, 0);
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
	using namespace nlohmann;
	json rootObject = json::object();
	json pointsArray = json::array();
	json geometryArray = json::array();

	for (const auto& obj : m_sceneObjects)
	{
		if (obj->type == ObjectType::Point)
			pointsArray.push_back(obj->Serialize());
		else
			geometryArray.push_back(obj->Serialize());
	}

	rootObject["points"] = std::move(pointsArray);
	rootObject["geometry"] = std::move(geometryArray);

	std::ofstream file(filePath);
	if (file.is_open())
	{
		file << rootObject.dump(4);
		file.close();
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
	std::ifstream file(filePath);
	if (!file.is_open())
		return;

	nlohmann::json rootObject;
	try
	{
		file >> rootObject;
	}
	catch (const std::exception& e)
	{
		MessageBoxA(nullptr, e.what(), "JSON Parse Error", MB_OK | MB_ICONERROR);
		return;
	}

	ClearSelection();
	m_sceneObjects.clear();
	m_currentFilePath = filePath;

	std::unordered_map<unsigned int, std::shared_ptr<Point>> pointMap;

	auto parseNameData = [](const nlohmann::json& jsonNode) -> std::optional<ParsedNameData> {
		if (jsonNode.contains("name"))
		{
			std::string n = jsonNode["name"];
			if (!n.empty())
			{
				unsigned int idx = ExtractIndexFromName(n);
				return ParsedNameData{ std::move(n), idx };
			}
		}
		return std::nullopt;
		};

	if (rootObject.contains("points"))
	{
		for (const auto& ptJson : rootObject["points"])
		{
			unsigned int id = ptJson["id"];
			float3 pos = ptJson["position"];
			std::optional<ParsedNameData> nameData = parseNameData(ptJson);

			auto pt = std::make_shared<Point>(id, pos, std::move(nameData));
			pointMap[id] = pt;
			m_sceneObjects.push_back(pt);
		}
	}

	if (rootObject.contains("geometry"))
	{
		for (const auto& geomJson : rootObject["geometry"])
		{
			unsigned int id = geomJson["id"];
			std::string type = geomJson["objectType"];
			std::optional<ParsedNameData> nameData = parseNameData(geomJson);

			std::vector<std::weak_ptr<Point>> controlPoints;
			if (geomJson.contains("controlPoints"))
			{
				for (const auto& cpRef : geomJson["controlPoints"])
				{
					unsigned int cpId = cpRef["id"];
					if (pointMap.count(cpId))
						controlPoints.push_back(pointMap[cpId]);
				}
			}

			std::shared_ptr<SceneObject> newObject = nullptr;
			auto assignCurve = [&]<typename CurveType>()
			{
				auto newCurve = std::make_shared<CurveType>(id, std::move(controlPoints), std::move(nameData));
				for (const auto& cpWeak : newCurve->m_controlPoints)
				{
					if (auto cp = cpWeak.lock())
						cp->AddDependent(newCurve);
				}
				newObject = std::move(newCurve);
			};

			if (type == BezierSurface::SchemaName || type == BSplineSurface::SchemaName)
			{
				uint2 size = geomJson["size"];
				uint2 samples = geomJson["samples"];

				std::shared_ptr<Surface> newSurface;
				auto assignSurface = [&]<typename SurfaceType>()
				{
					newSurface = std::make_shared<SurfaceType>(id, size, samples, std::move(controlPoints), std::move(nameData));
				};
				if (type == BezierSurface::SchemaName)
					assignSurface.operator() < BezierSurface > ();
				else
					assignSurface.operator() < BSplineSurface > ();


				for (const auto& cpWeak : newSurface->m_controlPoints)
				{
					if (auto cpShared = cpWeak.lock())
					{
						cpShared->AddDependent(newSurface);
						cpShared->m_surfaceLockCount++;
					}
				}

				newSurface->InitGeometry(m_device);
				newObject = std::move(newSurface);
			}
			else if (type == Torus::SchemaName)
			{
				float3 pos = geomJson["position"];
				float3 scale = geomJson["scale"];
				uint2 samples = geomJson["samples"];
				float largeRadius = geomJson["largeRadius"];
				float smallRadius = geomJson["smallRadius"];
				Quaternion rotation = geomJson["rotation"];
				Mat4f rotMatrix = Mat4f::FromQuaternion(rotation);
				newObject = std::make_shared<Torus>(id, pos, scale, rotMatrix, largeRadius, smallRadius, samples, std::move(nameData));
			}
			else if (type == BezierCurve::SchemaName)
				assignCurve.operator() < BezierCurve > ();
			else if (type == BSplineCurve::SchemaName)
				assignCurve.operator() < BSplineCurve > ();
			else if (type == InterpolatingCurve::SchemaName)
				assignCurve.operator() < InterpolatingCurve > ();

			if (newObject)
				m_sceneObjects.push_back(newObject);
		}
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
	std::vector<SceneObject*> selectedObjects;
	for (const auto& obj : m_sceneObjects)
	{
		if (!obj->selected)
			continue;
		if (obj->type == ObjectType::Torus || obj->IsA(ObjectType::Surface))
			selectedObjects.push_back(obj.get());
	}

	if (selectedObjects.size() != 2)
		return;

	auto obj1 = selectedObjects[0];
	auto obj2 = selectedObjects[1];

	float maxU1 = 1.0f;
	float maxV1 = 1.0f;
	float maxU2 = 1.0f;
	float maxV2 = 1.0f;

	if (auto surf1 = obj1->As<Surface>())
	{
		maxU1 = static_cast<float>(surf1->GetSegmentsU());
		maxV1 = static_cast<float>(surf1->GetSegmentsV());
	}
	if (auto surf2 = obj2->As<Surface>())
	{
		maxU2 = static_cast<float>(surf2->GetSegmentsU());
		maxV2 = static_cast<float>(surf2->GetSegmentsV());
	}

	// =============================================================
	// STAGE 1: GRID SEARCH (find closest points on coarse grid)
	// =============================================================
	constexpr int sampleCount = 20;
	struct SamplePoint { MathLib::Vec3f p; float u, v; };
	std::vector<SamplePoint> samples1, samples2;
	samples1.reserve((sampleCount + 1) * (sampleCount + 1));
	samples2.reserve((sampleCount + 1) * (sampleCount + 1));

	// Sample Surface 1
	for (int i = 0; i <= sampleCount; i++)
	{
		float u = (i / static_cast<float>(sampleCount)) * maxU1;
		for (int j = 0; j <= sampleCount; j++)
		{
			float v = (j / static_cast<float>(sampleCount)) * maxV1;
			auto res = EvaluateParametric(obj1, u, v);
			samples1.push_back({ res.p, u, v });
		}
	}

	// Sample Surface 2
	for (int i = 0; i <= sampleCount; i++)
	{
		float u = (i / static_cast<float>(sampleCount)) * maxU2;
		for (int j = 0; j <= sampleCount; j++)
		{
			float v = (j / static_cast<float>(sampleCount)) * maxV2;
			auto res = EvaluateParametric(obj2, u, v);
			samples2.push_back({ res.p, u, v });
		}
	}

	Vec4f currentParams{ 0.0f, 0.0f, 0.0f, 0.0f };

	if (m_useCursorAsHint)
	{
		float minVal1 = std::numeric_limits<float>::max();
		float minVal2 = std::numeric_limits<float>::max();
		Vec3f cursorPos(m_cursorPosition.x, m_cursorPosition.y, m_cursorPosition.z);

		for (const auto& s1 : samples1)
		{
			float dist = (s1.p - cursorPos).length_sqr();
			if (dist < minVal1)
			{
				minVal1 = dist;
				currentParams.x = s1.u;
				currentParams.y = s1.v;
			}
		}

		for (const auto& s2 : samples2)
		{
			float dist = (s2.p - cursorPos).length_sqr();
			if (dist < minVal2)
			{
				minVal2 = dist;
				currentParams.z = s2.u;
				currentParams.w = s2.v;
			}
		}
	}
	else
	{
		float minVal = std::numeric_limits<float>::max();

		for (const auto& s1 : samples1)
		{
			for (const auto& s2 : samples2)
			{
				float dist = (s1.p - s2.p).length_sqr();
				if (dist < minVal)
				{
					minVal = dist;
					currentParams = Vec4f(s1.u, s1.v, s2.u, s2.v);
				}
			}
		}
	}
	// =============================================================
	// STAGE 2: GRADIENT DESCENT
	// =============================================================
	float alpha = 0.01f;
	constexpr float alphaIncrease = 1.2f;
	constexpr float alphaReduction = 0.5f;
	constexpr float tolerance = 1e-6f;
	constexpr int maxIterations = 100;
	auto eval1 = EvaluateParametric(obj1, currentParams.x, currentParams.y);
	auto eval2 = EvaluateParametric(obj2, currentParams.z, currentParams.w);

	for (int iter = 0; iter < maxIterations; iter++)
	{
		Vec3f D = eval1.p - eval2.p;
		float distSqr = D.length_sqr();
		if (distSqr < tolerance)
			break;

		float gu = Vec3f::dot(D, eval1.du);
		float gv = Vec3f::dot(D, eval1.dv);
		float gs = Vec3f::dot(-D, eval2.du);
		float gt = Vec3f::dot(-D, eval2.dv);

		Vec4f nextParams = currentParams - alpha * Vec4f(gu, gv, gs, gt);

		auto applyBounds = [](float& val, float maxVal, SceneObject* obj) {
			if (obj->type == ObjectType::Torus)
			{
				val = std::fmod(val, maxVal);
				if (val < 0.0f) val += maxVal;
			}
			else
			{
				val = std::clamp(val, 0.0f, maxVal);
			}
			};

		applyBounds(nextParams.x, maxU1, obj1);
		applyBounds(nextParams.y, maxV1, obj1);
		applyBounds(nextParams.z, maxU2, obj2);
		applyBounds(nextParams.w, maxV2, obj2);

		auto nextEval1 = EvaluateParametric(obj1, nextParams.x, nextParams.y);
		auto nextEval2 = EvaluateParametric(obj2, nextParams.z, nextParams.w);

		if ((nextEval1.p - nextEval2.p).length_sqr() < distSqr)
		{
			currentParams = nextParams;
			eval1 = nextEval1;
			eval2 = nextEval2;
			alpha *= alphaIncrease;
		}
		else
		{
			alpha *= alphaReduction;
		}
	}

	m_intersectionStartParams = currentParams;
	eval1 = EvaluateParametric(obj1, currentParams.x, currentParams.y);
	eval2 = EvaluateParametric(obj2, currentParams.z, currentParams.w);
	float d = m_intersectionStep;
	if ((eval1.p - eval2.p).length_sqr() > 1e-5)
		return;

	// =============================================================
	// STAGE 3: TRACE THE INTERSECTION CURVE
	// =============================================================
	auto isOutOfBounds = [&](const Vec4f& params) {
		return params.x < 0.0f || params.x > maxU1 || params.y < 0.0f || params.y > maxV1 ||
			params.z < 0.0f || params.z > maxU2 || params.w < 0.0f || params.w > maxV2;
		};

	Vec3f startPos = eval1.p;

	std::vector<Vec3f> forwardPoints;
	std::vector<Vec3f> backwardPoints;
	bool isClosedLoop = false;
	bool forwardHitEdge = false;
	bool backwardHitEdge = false;

	for (float direction : {1.0f, -1.0f })
	{
		if (isClosedLoop)
			break;

		Vec4f marchingParams = currentParams;
		Vec3f startTangent, prevTangent;
		int stepCount = 0;
		while (true)
		{
			auto [p1, du1, dv1] = EvaluateParametric(obj1, marchingParams.x, marchingParams.y);
			auto [p2, du2, dv2] = EvaluateParametric(obj2, marchingParams.z, marchingParams.w);

			Vec3f np = Vec3f::cross(du1, dv1).normalize();
			Vec3f nq = Vec3f::cross(du2, dv2).normalize();
			Vec3f t = Vec3f::cross(np, nq);

			if (t.length_sqr() < 1e-8f)
				break;

			t = t.normalize();

			if (stepCount == 0)
			{
				t *= direction;
				if (direction == 1.0f)
					startTangent = t;
			}
			else
			{
				if (Vec3f::dot(t, prevTangent) < 0.0f)
					t = -t;
			}
			prevTangent = t;

			auto nextParamsOpt = FindIntersectionNextPoint(obj1, obj2, marchingParams, p1, t, d);

			if (!nextParamsOpt)
				break;

			auto [v_p1, v_du1, v_dv1] = EvaluateParametric(obj1, nextParamsOpt->x, nextParamsOpt->y);
			auto [v_p2, v_du2, v_dv2] = EvaluateParametric(obj2, nextParamsOpt->z, nextParamsOpt->w);
			if ((v_p1 - v_p2).length_sqr() > 1e-5f)
				break;

			if (isOutOfBounds(*nextParamsOpt))
			{
				Vec4f P0 = marchingParams;       // Last valid parameter
				Vec4f P1 = *nextParamsOpt;       // Out-of-bounds parameter
				Vec4f Delta = P1 - P0;           // Full Newton step vector
				float T = 1.0f;					 // 100% of the step by default
				int hitDim = -1;
				float hitTarget = 0.0f;

				auto checkBoundary = [&](int dim, float maxVal) {
					float p0 = P0.f[dim];
					float p1 = P1.f[dim];
					float d = Delta.f[dim];
					if (std::abs(d) > 1e-8f) // Prevent division by zero
					{
						if (p1 > maxVal)
						{
							float t = (maxVal - p0) / d;
							if (t < T) { T = t; hitDim = dim; hitTarget = maxVal; }
						}
						else if (p1 < 0.0f)
						{
							float t = (0.0f - p0) / d;
							if (t < T) { T = t; hitDim = dim; hitTarget = 0.0f; }
						}
					}
					};

				checkBoundary(0, maxU1);
				checkBoundary(1, maxV1);
				checkBoundary(2, maxU2);
				checkBoundary(3, maxV2);

				if (hitDim != -1)
				{
					Vec4f guess = P0 + Delta * T;
					auto exactEdgeOpt = FindExactEdgePoint(obj1, obj2, guess, hitDim, hitTarget);

					Vec4f finalParams = guess;
					bool exactValid = false;

					if (exactEdgeOpt)
					{
						auto isStrictlyOutOfBounds = [&](const Vec4f& p) {
							constexpr float eps = 1e-4f;
							return p.x < -eps || p.x > maxU1 + eps ||
								p.y < -eps || p.y > maxV1 + eps ||
								p.z < -eps || p.z > maxU2 + eps ||
								p.w < -eps || p.w > maxV2 + eps;
							};
						if (!isStrictlyOutOfBounds(*exactEdgeOpt))
						{
							finalParams = *exactEdgeOpt;
							exactValid = true;
						}
					}
					if (!exactValid)
					{
						auto [g_p1, g_du1, g_dv1] = EvaluateParametric(obj1, guess.x, guess.y);
						auto [g_p2, g_du2, g_dv2] = EvaluateParametric(obj2, guess.z, guess.w);
						if ((g_p1 - g_p2).length_sqr() > 1e-5)
							break;
					}

					auto [edgePos, _u, _v] = EvaluateParametric(obj1, finalParams.x, finalParams.y);


					Vec4f oppositeParams = finalParams;
					float oppositeTarget = 0.0f;

					if (hitTarget <= 0.0f)
					{
						if (hitDim == 0) oppositeTarget = maxU1;
						else if (hitDim == 1) oppositeTarget = maxV1;
						else if (hitDim == 2) oppositeTarget = maxU2;
						else if (hitDim == 3) oppositeTarget = maxV2;
					}
					oppositeParams.f[hitDim] = oppositeTarget;


					Vec3f seamEdgePos, seamOppositePos;
					if (hitDim == 0 || hitDim == 1)
					{
						seamEdgePos = EvaluateParametric(obj1, finalParams.x, finalParams.y).p;
						seamOppositePos = EvaluateParametric(obj1, oppositeParams.x, oppositeParams.y).p;
					}
					else
					{
						seamEdgePos = EvaluateParametric(obj2, finalParams.z, finalParams.w).p;
						seamOppositePos = EvaluateParametric(obj2, oppositeParams.z, oppositeParams.w).p;
					}

					// If the opposite edge physically touches this edge, it's a seam
					if ((seamOppositePos - seamEdgePos).length_sqr() < 1e-6f)
					{
						if (direction == 1.0f)
							forwardPoints.push_back(edgePos);
						else
							backwardPoints.push_back(edgePos);

						marchingParams = oppositeParams;
						continue;
					}

					if (direction == 1.0f)
					{
						forwardPoints.push_back(edgePos);
						forwardHitEdge = true;
					}
					else
					{
						backwardPoints.push_back(edgePos);
						backwardHitEdge = true;
					}
				}

				break; // hit the edge, stop marching in this direction
			}

			marchingParams = *nextParamsOpt;
			auto [newPos, _u, _v] = EvaluateParametric(obj1, marchingParams.x, marchingParams.y);
			if (stepCount > 3 && (newPos - startPos).length_sqr() < (d * d) && Vec3f::dot(t, startTangent) > 0.0f)
			{
				isClosedLoop = true;
				if (direction == 1.0f)
					forwardPoints.push_back(startPos);
				break;
			}

			if (direction == 1.0f)
				forwardPoints.push_back(newPos);
			else
				backwardPoints.push_back(newPos);

			stepCount++;
		}
	}

	if (!isClosedLoop && (!forwardHitEdge || !backwardHitEdge))
		return;

	std::vector<Vec3f> intersectionPoints;
	intersectionPoints.reserve(backwardPoints.size() + forwardPoints.size() + 1);
	intersectionPoints.insert(intersectionPoints.end(), backwardPoints.rbegin(), backwardPoints.rend());
	intersectionPoints.push_back(startPos);
	intersectionPoints.insert(intersectionPoints.end(), forwardPoints.begin(), forwardPoints.end());

	auto intersection = std::make_shared<Intersection>();
	intersection->InitGeometry(intersectionPoints, m_device);
	m_sceneObjects.push_back(std::move(intersection));
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
		DrawListMenu(selectedCount, selectedPoints, selectedCurve.lock());

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

	if (m_isBoxSelecting)
	{
		ImDrawList* drawList = ImGui::GetForegroundDrawList();
		auto [minX, maxX] = std::minmax(m_boxSelectStart.x, m_boxSelectCurrent.x);
		auto [minY, maxY] = std::minmax(m_boxSelectStart.y, m_boxSelectCurrent.y);
		ImVec2 p_min(static_cast<float>(minX), static_cast<float>(minY));
		ImVec2 p_max(static_cast<float>(maxX), static_cast<float>(maxY));
		bool ctrlHeld = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
		bool shiftHeld = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;

		ImU32 fillCol = IM_COL32(0, 130, 255, 40);   // Default Blue
		ImU32 borderCol = IM_COL32(0, 130, 255, 220);

		if (ctrlHeld)
		{
			fillCol = IM_COL32(255, 50, 50, 40);   // Subtract Red
			borderCol = IM_COL32(255, 50, 50, 220);
		}
		else if (shiftHeld)
		{
			fillCol = IM_COL32(50, 220, 50, 40);   // Append Green
			borderCol = IM_COL32(50, 220, 50, 220);
		}

		drawList->AddRectFilled(p_min, p_max, fillCol);
		drawList->AddRect(p_min, p_max, borderCol);
	}

	if (m_showIntersectionPopup)
	{
		ImGui::Begin("Intersection Parameters", &m_showIntersectionPopup, ImGuiWindowFlags_AlwaysAutoResize);

		ImGui::Checkbox("Use cursor as hint", &m_useCursorAsHint);
		ImGui::InputFloat("d", &m_intersectionStep);
		ImGui::Spacing();
		if (ImGui::Button("OK", ImVec2(120, 0)))
		{
			m_showIntersectionPopup = false;
			ActionFindIntersection();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel", ImVec2(120, 0)))
			m_showIntersectionPopup = false;
		ImGui::End();

	}

	ImGui::Render();
}

void CadApplication::DrawListMenu(int selectedCount, int selectedPoints, std::shared_ptr<Curve> activeCurve)
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
		{
			activeCurve->m_controlPoints.push_back(newPoint);
			newPoint->AddDependent(activeCurve);
			activeCurve->MarkDirty();
		}
	}

	auto getSelectedPoints = [&]() {
		std::vector<std::weak_ptr<Point>> pts;
		for (const auto& obj : m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
				pts.push_back(std::static_pointer_cast<Point>(obj));
		}
		return pts;
		};

	auto createAndRegisterCurve = [&]<typename CurveType>()
	{
		auto pts = getSelectedPoints();
		auto newCurve = std::make_shared<CurveType>(std::move(pts));
		for (const auto& cpWeak : newCurve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->AddDependent(newCurve);
		}
		m_sceneObjects.push_back(newCurve);
	};

	ImGui::BeginDisabled(selectedPoints == 0);
	if (ImGui::Button("Add Bezier Curve"))
		createAndRegisterCurve.operator() < BezierCurve > ();
	if (ImGui::Button("Add B-Spline (C2) Curve"))
		createAndRegisterCurve.operator() < BSplineCurve > ();
	if (ImGui::Button("Add Interpolating Spline (C2)"))
		createAndRegisterCurve.operator() < InterpolatingCurve > ();
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



	ImGui::BeginDisabled(pointsToAdd.empty() || !activeCurve);
	if (ImGui::Button("Add Points to Curve"))
	{
		activeCurve->m_controlPoints.insert(activeCurve->m_controlPoints.end(), pointsToAdd.begin(), pointsToAdd.end());
		for (const auto& pt : pointsToAdd)
			pt->AddDependent(activeCurve);

		activeCurve->MarkDirty();
	}
	ImGui::EndDisabled();

	bool changed = false;
	if (ImGui::Button("Create C0 Surface"))
	{
		m_previewType = 0;
		changed = true;
		m_showSurfacePopup = true;
	}
	if (ImGui::Button("Create C2 Surface"))
	{
		m_previewType = 1;
		changed = true;
		m_showSurfacePopup = true;
	}

	if (m_showSurfacePopup)
	{
		const char* popupTitle = (m_previewType == 0) ? "C0 Surface Parameters" : "C2 Surface Parameters";
		ImGui::Begin(popupTitle, &m_showSurfacePopup);


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
			m_surfaceBuilder.UpdateGeometry(
				m_device,
				(m_previewType == 0) ? SurfaceType::C0 : SurfaceType::C2,
				static_cast<SurfaceShape>(m_previewShape),
				m_previewSegU,
				m_previewSegV,
				(m_previewShape == 0) ? m_previewWidth : m_previewRadius,
				m_previewDim2,
				m_cursorPosition
			);
		}

		if (ImGui::Button("Add to Scene"))
		{
			auto result = m_surfaceBuilder.Build(m_device);
			std::shared_ptr<Surface> sharedSurface = std::move(result.surface);
			for (auto& pt : result.points)
			{
				pt->AddDependent(sharedSurface);
				m_sceneObjects.push_back(pt);
			}

			m_sceneObjects.push_back(sharedSurface);
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
		std::erase_if(curve->m_controlPoints, [curve](const auto& cpWeak) {
			if (auto cp = cpWeak.lock())
			{
				if (cp->selected)
				{
					cp->selected = false;
					cp->RemoveDependent(curve);
					return true;
				}
				return false;
			}
			return true;
			});
		curve->MarkDirty();
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
		unsigned int pointsU = surface->m_gridPointsU;
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
				cp->m_surfaceLockCount = std::max(0, cp->m_surfaceLockCount - 1);
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
		else if (auto patch = selectedObj->As<GregoryPatch>())
			DrawPatchMenu(*patch);
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
	ImGui::Separator();
	ImGui::Text("Scale");

	if (ImGui::DragFloat3("Scale XYZ", &torus.m_scale.x, 0.01f, Torus::cMinScale, Torus::cMaxScale))
		transformChanged = true;

	float uniformScale = torus.m_scale.x;
	if (ImGui::DragFloat("Uniform Scale", &uniformScale, 0.01f, Torus::cMinScale, Torus::cMaxScale))
	{
		torus.m_scale = { uniformScale, uniformScale, uniformScale };
		transformChanged = true;
	}

	//if (ImGui::DragFloat("Scale", &torus.m_scale, 0.01f, Torus::cMinScale, Torus::cMaxScale))
	//	transformChanged = true;

	if (transformChanged)
		torus.UpdateModelMatrix();
}

void CadApplication::DrawPointMenu(Point& selectedObj)
{
	if (ImGui::DragFloat3("Position", &selectedObj.m_position.x, 0.01f))
	{
		m_selectionDirty = true;
		selectedObj.NotifyDependents();
	}
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

void CadApplication::DrawPatchMenu(GregoryPatch& patch)
{
	ImGui::Text("Tessellation Parameters");
	ImGui::Spacing();
	// Sliders to control the shader density
	ImGui::SliderInt("Lines per Segment (U)", &patch.m_linesPerSegmentU, 2, 64);
	ImGui::SliderInt("Lines per Segment (V)", &patch.m_linesPerSegmentV, 2, 64);
	ImGui::SliderInt("Smoothness", &patch.m_smoothness, 2, 64);
	ImGui::Separator();
}

void CadApplication::DrawActionCombo()
{
	ImGui::Text("Interactive Action");
	const char* actions[] = {
		"None", "Free Translation", "Translate X", "Translate Y", "Translate Z",
		"Free Arcball", "Rotate X", "Rotate Y", "Rotate Z", "Scale", "Scale X", "Scale Y", "Scale Z"
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
