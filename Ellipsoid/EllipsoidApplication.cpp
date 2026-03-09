#include "pch.h"
#include "EllipsoidApplication.h"
#include "DxStructures.h"
#include "../MathLib/MathLib.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

using namespace std;
using namespace MathLib;
using Microsoft::WRL::ComPtr;

pair<double, double> CalculateCoordsFromPixel(double x, double y, double width, double height);

Vec3d Ellipsoid::color = MathLib::Vec3d(1.0, 1.0, 0.0); // Yellow color for the elipsoid.

void Ellipsoid::UpdateDMprimMatrix()
{
	if (!needsUpdate)
		return;

	Mat4d invScale = Mat4d::Scaling(1.0 / scale.x, 1.0 / scale.y, 1.0 / scale.z);
	Mat4d invRot = rotationMatrix.Transpose();
	Mat4d invTrans = Mat4d::Translation(-position.x, -position.y, -position.z);

	Mat4d MInverse = invScale * (invRot * invTrans);
	Mat4d MInverseT = MInverse.Transpose();

	//Mat4d MInverse = Mat4d::CreateInverseTRS(position, rotation, scale);
	//Mat4d MInverseT = MInverse.Transpose();
	Mat4d D = Mat4d::Diagonal(1.0 / (radii.x * radii.x), 1.0 / (radii.y * radii.y), 1.0 / (radii.z * radii.z), -1.0);

	DMprim = MInverseT * D * MInverse;
	//DMprim = D;
	needsUpdate = false;
}


EllipsoidApplication::EllipsoidApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	: DxApplication(hInstance, wndWidth, wndHeight, wndTitle)
{
	SIZE wndSize = m_window.getClientSize();
	Texture2DDescription cpuTextureDesc(wndSize);
	m_cpuTexture = m_device.CreateTexture2D(cpuTextureDesc);
	m_cpuTextureView = m_device.CreateShaderResourceView(m_cpuTexture);
	

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

	ImGui_ImplWin32_Init(m_window.getHandle());
	ImGui_ImplDX11_Init(m_device.get(), m_device.getContext().Get());
}

EllipsoidApplication::~EllipsoidApplication()
{
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();
}

bool EllipsoidApplication::ProcessMessage(WindowMessage& msg)
{
	if (ImGui::GetCurrentContext() == nullptr)
		return WindowApplication::ProcessMessage(msg);

	if (ImGui_ImplWin32_WndProcHandler(m_window.getHandle(), msg.message, msg.wParam, msg.lParam))
		return true;

	ImGuiIO& io = ImGui::GetIO();
	if (io.WantCaptureMouse)
	{
		if (msg.message == WM_LBUTTONDOWN || msg.message == WM_RBUTTONDOWN ||
			msg.message == WM_MOUSEMOVE || msg.message == WM_MOUSEWHEEL)
		{
			return true;
		}
	}

	int xPos = (int)(short)LOWORD(msg.lParam);
	int yPos = (int)(short)HIWORD(msg.lParam);

	switch (msg.message)
	{
	case WM_GETMINMAXINFO:
	{
		MINMAXINFO* mmi = reinterpret_cast<MINMAXINFO*>(msg.lParam);
		mmi->ptMinTrackSize.x = m_minWidth;
		mmi->ptMinTrackSize.y = m_minHeight;
		msg.result = 0;
		return true;
	}
	case WM_ERASEBKGND:
		msg.result = 1;
		return true; // Prevent flickering by not erasing the background.
	case WM_LBUTTONDOWN:
		m_interactionMode = InteractionMode::Rotating;
		m_lastMousePos = { xPos, yPos };
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
				Mat4d rotX = Mat4d::RotationX(dy * 0.01); // Rotate around x-axis based on vertical mouse movement.
				Mat4d rotY = Mat4d::RotationY(dx * 0.01); // Rotate around y-axis based on horizontal mouse movement.
				//m_ellipsoid.rotation.y += dx * 0.01; // Rotate around y-axis.
				//m_ellipsoid.rotation.x += dy * 0.01; // Rotate around x-axis.
				m_ellipsoid.rotationMatrix = rotY * rotX * m_ellipsoid.rotationMatrix;
				m_ellipsoid.rotationMatrix.Orthonormalize3x3(); // Keep the rotation matrix orthonormal to prevent distortion.
			}
			else if (m_interactionMode == InteractionMode::Translating)
			{
				double worldUnitsPerPixelX = (2.0 * m_viewport.aspectRatio) / m_viewport.width;
				double worldUnitsPerPixelY = 2.0 / m_viewport.height;
				m_ellipsoid.position.x += dx * worldUnitsPerPixelX; // Move along x-axis.
				m_ellipsoid.position.y -= dy * worldUnitsPerPixelY; // Move along y-axis (inverted).
			}
			m_lastMousePos = { xPos, yPos };
			m_ellipsoid.needsUpdate = true;
			m_currentStep = m_initialStep;
			return true;
		}
		break;
	case WM_MOUSEWHEEL:
	{
		short zDelta = (short)HIWORD(msg.wParam);
		WORD fwKeys = LOWORD(msg.wParam);

		if (fwKeys & MK_SHIFT)
		{
			// Rotate around z-axis.
			//m_ellipsoid.rotation.z += (zDelta > 0) ? 0.1 : -0.1;
			Mat4d rotZ = Mat4d::RotationZ((zDelta > 0) ? 0.1 : -0.1);
			m_ellipsoid.rotationMatrix = rotZ * m_ellipsoid.rotationMatrix;
			m_ellipsoid.rotationMatrix.Orthonormalize3x3(); // Keep the rotation matrix orthonormal to prevent distortion.
		}
		else
		{
			float scaleFactor = (zDelta > 0) ? 1.1f : 0.9f;
			m_ellipsoid.scale *= scaleFactor;
		}
		m_ellipsoid.needsUpdate = true;
		m_currentStep = m_initialStep;
		return true;
	}
	}

	return DxApplication::ProcessMessage(msg);
}

void EllipsoidApplication::Render()
{
	m_ellipsoid.UpdateDMprimMatrix();
	SIZE clientSize = m_window.getClientSize();

	LONG width = clientSize.cx;
	LONG height = clientSize.cy;

	if (width == 0 || height == 0)
		return;
	
	DrawEllipsoid();
	DrawMenu(width, height);

	const float clear_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	m_device.getContext()->ClearRenderTargetView(m_backBuffer.Get(), clear_color);
	m_device.getContext()->OMSetRenderTargets(1, m_backBuffer.GetAddressOf(), nullptr);
	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
}

void EllipsoidApplication::UpdateResources(int width, int height)
{
	UpdateViewport(width, height);

	Texture2DDescription cpuTextureDesc(
		static_cast<UINT>(m_viewport.width),
		static_cast<UINT>(m_viewport.height)
	);
	m_cpuTexture = m_device.CreateTexture2D(cpuTextureDesc);
	m_cpuTextureView = m_device.CreateShaderResourceView(m_cpuTexture);

	// Resize buffer used for CPU rendering
	//size_t requiredSize = static_cast<size_t>(width * height);
	size_t requiredSize = static_cast<size_t>(m_viewport.width * m_viewport.height);
	m_pixelData.assign(requiredSize, 0xFF000000);

	m_currentStep = m_initialStep;
}

void EllipsoidApplication::DrawEllipsoid()
{
	if (m_currentStep < 1)
		return;

	int width = static_cast<int>(m_viewport.width);
	int height = static_cast<int>(m_viewport.height);
	#pragma omp parallel for
	for (int i = 0; i < width; i += m_currentStep)
	{
		#pragma omp parallel for
		for (int j = 0; j < height; j += m_currentStep)
		{
			uint32_t finalPixelColor = ImGui::ColorConvertFloat4ToU32(*reinterpret_cast<ImVec4*>(m_backgroundColor));
			double sampleX = i + m_currentStep / 2.0; // Sample at the center of the block for better visual results.
			double sampleY = j + m_currentStep / 2.0;
			auto [x, y] = CalculateCoordsFromPixel(sampleX, sampleY, m_viewport.width, m_viewport.height);
			x *= m_viewport.aspectRatio; // Adjust x coordinate for aspect ratio.

			// w = z*k + p
			Vec4d k = Vec4d(0.0, 0.0, 1.0, 0.0);
			Vec4d p = Vec4d(x, y, 0.0, 1.0);

			double a = Vec4d::dot(k, m_ellipsoid.DMprim * k);
			double b = 2.0 * Vec4d::dot(k, m_ellipsoid.DMprim * p);
			double c = Vec4d::dot(p, m_ellipsoid.DMprim * p);

			auto solution = SolveQuadratic(a, b, c);
			if (solution.has_value())
			{
				double z = solution->max_root();

				Vec4d w = Vec4d(x, y, z, 1.0);
				Vec4d grad = 2.0 * (m_ellipsoid.DMprim * w);
				Vec3d normal = Vec3d(grad.x, grad.y, grad.z).normalize();

				double lightIntensity = std::max(0.0, Vec3d::dot(normal, Vec3d(0.0, 0.0, 1.0)));
				lightIntensity = std::pow(lightIntensity, m_specularExponent);
				Vec3d color = Ellipsoid::color * lightIntensity;

				uint32_t red = static_cast<uint32_t>(std::min(1.0, color.x) * 255.0);
				uint32_t green = static_cast<uint32_t>(std::min(1.0, color.y) * 255.0);
				uint32_t blue = static_cast<uint32_t>(std::min(1.0, color.z) * 255.0);
				finalPixelColor = red | (green << 8) | (blue << 16) | (0xFF << 24); // RGBA format.
			}

			for (int blockY = 0; blockY < m_currentStep && (j + blockY) < height; blockY++)
			{
				for (int blockX = 0; blockX < m_currentStep && (i + blockX) < width; blockX++)
				{
					//m_pixelData[(j + blockY) * totalWidth + (i + blockX)] = finalPixelColor;
					m_pixelData[(j + blockY) * width + (i + blockX)] = finalPixelColor;
				}
			}
		}
	}

	D3D11_MAPPED_SUBRESOURCE mappedResource;
	m_device.getContext()->Map(m_cpuTexture.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);

	uint8_t* dest = static_cast<uint8_t*>(mappedResource.pData);
	uint8_t* src = reinterpret_cast<uint8_t*>(m_pixelData.data());
	size_t srcRowBytes = width * sizeof(uint32_t);
	for (LONG row = 0; row < m_viewport.height; row++)
	{
		memcpy(dest, src, srcRowBytes);
		dest += mappedResource.RowPitch;
		src += srcRowBytes;
	}

	m_device.getContext()->Unmap(m_cpuTexture.Get(), 0);
	m_currentStep /= 2;
}

void EllipsoidApplication::DrawMenu(int width, int height)
{
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	float uv_x_max = static_cast<float>(m_viewport.width) / width;
	ImGui::GetBackgroundDrawList()->AddImage(
		(ImTextureID)m_cpuTextureView.Get(),
		ImVec2(0, 0),
		ImVec2(static_cast<float>(m_viewport.width), static_cast<float>(height))
	);

	ImGui::SetNextWindowPos(ImVec2(static_cast<float>(m_viewport.width), 0.0f));
	ImGui::SetNextWindowSize(ImVec2(static_cast<float>(menuWidth), static_cast<float>(height)));

	// Use flags to make it act like a fixed side-panel
	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse |
		ImGuiWindowFlags_NoTitleBar;

	ImGui::Begin("Settings", nullptr, windowFlags);
	ImGui::Text("Ellipsoid Settings");
	ImGui::Separator();
	float tempRadii[3] = { static_cast<float>(m_ellipsoid.radii.x), static_cast<float>(m_ellipsoid.radii.y), static_cast<float>(m_ellipsoid.radii.z) };
	if (ImGui::DragFloat3("Radii", tempRadii, 0.1f, 0.1f, 100.0f))
	{
		m_ellipsoid.radii = Vec3d(tempRadii[0], tempRadii[1], tempRadii[2]);
		m_ellipsoid.needsUpdate = true;
		m_currentStep = m_initialStep;
	}
	if (ImGui::SliderInt("Specular Exponent", &m_specularExponent, 1, 32))
	{
		m_currentStep = m_initialStep;
	}

	ImGui::Separator();
	ImGui::Text("Appearance");
	if (ImGui::ColorEdit3("Background Color", m_backgroundColor))
		m_currentStep = m_initialStep;


	ImGui::Separator();
	ImGui::Text("Rendering Performance");
	static int exponent = cInitialStepExponent;
	char sliderLabel[32];
	sprintf_s(sliderLabel, "Step: %d", m_initialStep);
	if (ImGui::SliderInt("Initial Step", &exponent, 0, cMaxStepExponent, sliderLabel))
	{
		m_initialStep = 1 << exponent; // 2^exponent
	}

	ImGui::Text("Current Step: %d", m_currentStep * 2);

	ImGui::End();
	ImGui::Render();
}

void EllipsoidApplication::UpdateViewport(int width, int height)
{
	m_viewport.width = static_cast<double>(width - menuWidth);
	m_viewport.height = static_cast<double>(height);
	m_viewport.aspectRatio = m_viewport.width / m_viewport.height;
}

pair<double, double> CalculateCoordsFromPixel(double x, double y, double width, double height)
{
	double xCoord = (x / width) * 2.0 - 1.0; // Map to [-1, 1]
	double yCoord = 1.0 - (y / height) * 2.0; // Map to [1, -1] (inverted y-axis)
	return { xCoord, yCoord };
}