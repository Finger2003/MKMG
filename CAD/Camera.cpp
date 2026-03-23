#include "pch.h"
#include "Camera.h"

using namespace std;
using namespace MathLib;
void Camera::SetProjection(float fovY, float aspectRatio, float nearPlane, float farPlane)
{
	m_projMatrix = Mat4f::Perspective(fovY, aspectRatio, nearPlane, farPlane);
}

void Camera::Orbit(float deltaX, float deltaY)
{
	m_yaw += deltaX;
	m_pitch += deltaY;
	m_pitch = std::clamp(m_pitch, -std::numbers::pi_v<float> / 2.0f + 0.01f, std::numbers::pi_v<float> / 2.0f - 0.01f);
}

void Camera::Pan(float deltaX, float deltaY)
{
	float cosY = std::cos(m_yaw);
	float sinY = std::sin(m_yaw);
	float cosP = std::cos(m_pitch);
	float sinP = std::sin(m_pitch);

	Vec3f right(cosY, 0.0f, sinY);
	Vec3f up(sinY * sinP, cosP, -cosY * sinP);

	m_target += right * deltaX + up * deltaY;
}

void Camera::Zoom(float delta)
{
	m_distance -= delta;
	m_distance = std::max(m_distance, 0.1f);
}

MathLib::Mat4f Camera::GetViewMatrix() const
{
	Mat4f translationToTarget = Mat4f::Translation(-m_target.x, -m_target.y, -m_target.z);
	Mat4f rotY = Mat4f::RotationY(m_yaw);
	Mat4f rotX = Mat4f::RotationX(m_pitch);
	Mat4f pushAway = Mat4f::Translation(0.0f, 0.0f, -m_distance);

	return pushAway * rotX * rotY * translationToTarget;
}

MathLib::Mat4f Camera::GetInverseViewMatrix() const
{
	Mat4f translationFromTarget = Mat4f::Translation(m_target.x, m_target.y, m_target.z);
	Mat4f rotY = Mat4f::RotationY(-m_yaw);
	Mat4f rotX = Mat4f::RotationX(-m_pitch);
	Mat4f pullBack = Mat4f::Translation(0.0f, 0.0f, m_distance);

	return translationFromTarget * rotY * rotX * pullBack;
}
