#include "pch.h"
#include "Camera.h"

using namespace std;
using namespace MathLib;

void Camera::Orbit(float deltaX, float deltaY)
{
	float newYaw = m_yaw + deltaX;
	float newPitch = std::clamp(m_pitch + deltaY, -std::numbers::pi_v<float> / 2.0f + 0.01f, std::numbers::pi_v<float> / 2.0f - 0.01f);

	if (m_yaw != newYaw || m_pitch != newPitch)
	{
		m_yaw = newYaw;
		m_pitch = newPitch;
		m_viewDirty = true;
	}
}

void Camera::Pan(float deltaX, float deltaY)
{
	Vec3f right = GetRightVector();
	Vec3f up = GetUpVector();

	Vec3f newTarget = m_target + right * deltaX + up * deltaY;
	if (m_target.x != newTarget.x || m_target.y != newTarget.y || m_target.z != newTarget.z)
	{
		m_target = newTarget;
		m_viewDirty = true;
	}
}

void Camera::Zoom(float delta)
{
	float newDistance = std::max(m_distance - delta, 0.1f);
	if (m_distance != newDistance)
	{
		m_distance = newDistance;
		m_viewDirty = true;
	}
}


MathLib::Vec3f Camera::GetRightVector() const
{
	return Vec3f(m_invViewMatrix.m[0][0], m_invViewMatrix.m[1][0], m_invViewMatrix.m[2][0]);
}

MathLib::Vec3f Camera::GetUpVector() const
{
	return Vec3f(m_invViewMatrix.m[0][1], m_invViewMatrix.m[1][1], m_invViewMatrix.m[2][1]);
}

MathLib::Vec3f Camera::GetForwardVector() const
{
	return Vec3f(-m_invViewMatrix.m[0][2], -m_invViewMatrix.m[1][2], -m_invViewMatrix.m[2][2]);
}

MathLib::Vec3f Camera::GetPosition() const
{
	return Vec3f(m_invViewMatrix.m[0][3], m_invViewMatrix.m[1][3], m_invViewMatrix.m[2][3]);
}

bool Camera::UpdateViewMatrices()
{
	if (!m_viewDirty)
		return false;

	Mat4f translationToTarget = Mat4f::Translation(-m_target.x, -m_target.y, -m_target.z);
	Mat4f rotY = Mat4f::RotationY(m_yaw);
	Mat4f rotX = Mat4f::RotationX(m_pitch);
	Mat4f pushAway = Mat4f::Translation(0.0f, 0.0f, -m_distance);
	m_viewMatrix = pushAway * rotX * rotY * translationToTarget;
	UpdateProjView();

	Mat4f translationFromTarget = Mat4f::Translation(m_target.x, m_target.y, m_target.z);
	Mat4f invRotY = rotY.Transpose();
	Mat4f invRotX = rotX.Transpose();
	//Mat4f invRotY = Mat4f::RotationY(-m_yaw);
	//Mat4f invRotX = Mat4f::RotationX(-m_pitch);
	Mat4f pullBack = Mat4f::Translation(0.0f, 0.0f, m_distance);
	m_invViewMatrix = translationFromTarget * invRotY * invRotX * pullBack;

	m_viewDirty = false;
	return true;
}

bool Camera::UpdateProjectionMatrix()
{
	if (!m_projDirty)
		return false;

	m_projMatrix = Mat4f::Perspective(m_fovY, m_aspectRatio, m_nearPlane, m_farPlane);
	UpdateProjView();

	m_panScaleFactor = 2.0f * std::tan(m_fovY / 2.0f) / m_viewportHeight;

	m_projDirty = false;
	return true;
}

void Camera::UpdateProjView()
{
	m_projViewMatrix = m_projMatrix * m_viewMatrix;
}

void Camera::SetFovY(float fovYRad)
{
	float clampedFov = std::clamp(fovYRad, 0.01f, 3.12f); // Assuming radians internally
	if (m_fovY != clampedFov)
	{
		m_fovY = clampedFov;
		m_projDirty = true;
	}
}

void Camera::SetPlanes(float nearPlane, float farPlane)
{
	float safeNear = std::max(nearPlane, 0.01f);
	float safeFar = std::max(farPlane, safeNear + 0.01f);

	if (m_nearPlane != safeNear || m_farPlane != safeFar)
	{
		m_nearPlane = safeNear;
		m_farPlane = safeFar;
		m_projDirty = true;
	}
}

void Camera::SetViewportSize(int width, int height)
{
	int safeWidth = std::max(width, 1);
	int safeHeight = std::max(height, 1);
	if (m_viewportWidth != safeWidth || m_viewportHeight != safeHeight)
	{
		m_viewportWidth = safeWidth;
		m_viewportHeight = safeHeight;
		m_aspectRatio = static_cast<float>(m_viewportWidth) / m_viewportHeight;
		m_projDirty = true;
	}
}

MathLib::Vec3f Camera::GetPositionAtDepth(float normX, float normY, float depth) const
{
	float planeHalfHeight = m_panScaleFactor * m_viewportHeight * depth / 2.0f;
	float planeHalfWidth = planeHalfHeight * m_aspectRatio;

	Vec4f localPos(normX * planeHalfWidth, normY * planeHalfHeight, -depth, 1.0f);
	Vec4f worldPos = m_invViewMatrix * localPos;

	return Vec3f::FromVec4f(worldPos);
}
