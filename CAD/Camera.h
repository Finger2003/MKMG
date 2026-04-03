#pragma once
#include "../MathLib/Mat4f.h"

class Camera
{
public:
	Camera(){ UpdateViewMatrices(); }

#pragma region Camera Controls
	void Orbit(float deltaX, float deltaY);
	void Pan(float deltaX, float deltaY);
	void Zoom(float delta);
#pragma endregion

#pragma region Matrix Getters
	MathLib::Mat4f GetViewMatrix() const { return m_viewMatrix; }
	MathLib::Mat4f GetInverseViewMatrix() const { return m_invViewMatrix; }
	MathLib::Mat4f GetProjectionMatrix() const { return m_projMatrix; }
	MathLib::Mat4f GetProjViewMatrix() const { return m_projViewMatrix; }
#pragma endregion

#pragma region View Parameters Getters
	float GetDistance() const { return m_distance; }
	MathLib::Vec3f GetRightVector() const;
	MathLib::Vec3f GetUpVector() const;
	MathLib::Vec3f GetForwardVector() const;
	MathLib::Vec3f GetPosition() const;
#pragma endregion

#pragma region Update Methods
	bool UpdateViewMatrices();
	bool UpdateProjectionMatrix();
	void UpdateProjView();
#pragma endregion

#pragma region Projection Setters/Getters
	void SetFovY(float fovYRad);
	void SetPlanes(float nearPlane, float farPlane);
	void SetViewportSize(int width, int height);
	float GetFovY() const { return m_fovY; }
	float GetNearPlane() const { return m_nearPlane; }
	float GetFarPlane() const { return m_farPlane; }
	float GetPanScaleFactor() const { return m_panScaleFactor; }
	float GetAspectRatio() const { return m_aspectRatio; }
#pragma endregion

	MathLib::Vec3f GetPositionOnFocalPlane(float normX, float normY) const { return GetPositionAtDepth(normX, normY, m_distance); }
	MathLib::Vec3f GetPositionAtDepth(float normX, float normY, float depth) const;
private:
#pragma region Matrices
	MathLib::Mat4f m_viewMatrix;
	MathLib::Mat4f m_invViewMatrix;
	MathLib::Mat4f m_projMatrix;
	MathLib::Mat4f m_projViewMatrix;
#pragma endregion

#pragma region View Parameters
	MathLib::Vec3f m_target{ 0.0f, 0.0f, 0.0f };
	float m_distance = 5.0f;
	float m_pitch = 0.0f; // Rotation around X-axis
	float m_yaw = 0.0f;   // Rotation around Y-axis
#pragma endregion

#pragma region Projection Parameters
	float m_fovY = 60.0f * (std::numbers::pi_v<float> / 180.0f);
	float m_nearPlane = 0.1f;
	float m_farPlane = 100.0f;
	float m_aspectRatio = 1.0f;
	float m_panScaleFactor = 0.0f;
	int m_viewportWidth = 800;
	int m_viewportHeight = 600;
#pragma endregion

#pragma region Dirty Flags
	bool m_viewDirty = true;
	bool m_projDirty = true;
#pragma endregion
};

