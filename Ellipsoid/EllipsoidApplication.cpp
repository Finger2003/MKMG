#include "pch.h"
#include "EllipsoidApplication.h"
#include "../MathLib/MathLib.h"

using namespace std;
using namespace MathLib;
pair<double, double> CalculateCoordsFromPixel(int x, int y, int width, int height);
double CalculateCoordFromPixel(int pixel, int maxPixel);

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
	: WindowApplication(hInstance, wndWidth, wndHeight, wndTitle)
{
	m_bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	m_bitmapInfo.bmiHeader.biPlanes = 1;
	m_bitmapInfo.bmiHeader.biBitCount = 32;
	m_bitmapInfo.bmiHeader.biCompression = BI_RGB;
}

bool EllipsoidApplication::ProcessMessage(WindowMessage& msg)
{
	int xPos = (int)(short)LOWORD(msg.lParam);
	int yPos = (int)(short)HIWORD(msg.lParam);

	switch (msg.message)
	{
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
			}
			else if (m_interactionMode == InteractionMode::Translating)
			{
				// Translate the elipsoid based on mouse movement.
				m_ellipsoid.position.x += dx * 0.001; // Move along x-axis.
				m_ellipsoid.position.y -= dy * 0.001; // Move along y-axis (inverted).
			}
			m_lastMousePos = { xPos, yPos };
			m_ellipsoid.needsUpdate = true;
			m_step = minStep;
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
		}
		else
		{
			float scaleFactor = (zDelta > 0) ? 1.1f : 0.9f;
			m_ellipsoid.scale *= scaleFactor;
		}
		m_ellipsoid.needsUpdate = true;
		m_step = minStep;
		return true;
	}
	}

	return WindowApplication::ProcessMessage(msg);
}

int EllipsoidApplication::MainLoop()
{
	MSG msg{};
	do
	{
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			Render();
		}
	} while (msg.message != WM_QUIT);
	return msg.wParam;
}

void EllipsoidApplication::Render()
{
	m_ellipsoid.UpdateDMprimMatrix();
	SIZE clientSize = m_window.getClientSize();
	LONG width = clientSize.cx;
	LONG height = clientSize.cy;

	double aspectRatio = static_cast<double>(width) / height;

	size_t requiredSize = static_cast<size_t>(width * height);
	if (m_pixelData.size() != requiredSize)
	{
		m_pixelData.resize(requiredSize, 0xFF000000); // Initialize with opaque black.
		m_bitmapInfo.bmiHeader.biWidth = width;
		m_bitmapInfo.bmiHeader.biHeight = height; // Negative height for top-down bitmap.
	}


	for (LONG i = 0; i < width; i += m_step)
	{
		for (LONG j = 0; j < height; j += m_step)
		{
			uint32_t finalPixelColor = 0;

			double x = CalculateCoordFromPixel(i, width) * aspectRatio; // Adjust x coordinate for aspect ratio.
			double y = CalculateCoordFromPixel(j, height);

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

				finalPixelColor = (0xFF << 24) | (red << 16) | (green << 8) | blue; // ARGB format.
			}
			for (LONG blockY = 0; blockY < m_step && (j + blockY) < height; blockY++)
			{
				for (LONG blockX = 0; blockX < m_step && (i + blockX) < width; blockX++)
				{
					m_pixelData[(j + blockY) * width + (i + blockX)] = finalPixelColor;
				}
			}
		}
	}

	HWND hWnd = m_window.getHandle();
	HDC hdc = GetDC(hWnd);
	SetDIBitsToDevice(hdc, 0, 0, width, height, 0, 0, 0, height, m_pixelData.data(), &m_bitmapInfo, DIB_RGB_COLORS);
	ReleaseDC(hWnd, hdc);

	m_step = std::max(m_step / 2, 1);
}

pair<double, double> CalculateCoordsFromPixel(int x, int y, int width, int height)
{
	double xCoord = (x / (double)width) * 2.0 - 1.0; // Map to [-1, 1]
	double yCoord = (y / (double)height) * 2.0 - 1.0; // Map to [-1, 1]
	//double yCoord = 1.0 - (y / (double)height) * 2.0; // Map to [1, -1] (inverted y-axis)
	return { xCoord, yCoord };
}

double CalculateCoordFromPixel(int pixel, int maxPixel)
{
	return (pixel / (double)maxPixel) * 2.0 - 1.0; // Map to [-1, 1]
}
