#pragma once
#include "../MathLib/Mat4f.h"

class Camera
{
public:
	void SetProjection(float fovY, float aspectRatio, float nearPlane, float farPlane);

	void Orbit(float deltaX, float deltaY);
	void Pan(float deltaX, float deltaY);
	void Zoom(float delta);

	MathLib::Mat4f GetViewMatrix() const;// { return m_viewMatrix; }
	MathLib::Mat4f GetInverseViewMatrix() const;
	MathLib::Mat4f GetProjectionMatrix() const { return m_projMatrix; }
	float GetDistance() const { return m_distance; }

	MathLib::Vec3f GetRightVector() const;
	MathLib::Vec3f GetUpVector() const;
	MathLib::Vec3f GetForwardVector() const;
	MathLib::Vec3f GetPosition() const;

private:
	MathLib::Vec3f m_target{ 0.0f, 0.0f, 0.0f };
	float m_distance = 5.0f;
	float m_pitch = 0.0f; // Rotation around X-axis
	float m_yaw = 0.0f;   // Rotation around Y-axis

	MathLib::Mat4f m_projMatrix = MathLib::Mat4f::Identity();
};

