#pragma once
#include "WindowApplication.h"
#include "../MathLib/Vec3d.h"
#include "../MathLib/Mat4d.h"

//struct Elipsoid
//{
//	double rx{ 1.0 }, ry{ 1.0 }, rz{ 1.0 }; // Radii along x, y and z axes.
//	double posx{}, posy{}, posz{}; // Position of the center of the elipsoid.
//	double rotX{ 0.0 }, rotY{ 0.0 }, rotZ{ 0.0 }; // Rotation angles around x, y and z axes.
//	double scale = 1.0;
//
//	static constexpr MathLib::Vec3d color = MathLib::Vec3d(1.0, 1.0, 0.0);
//};
struct Ellipsoid
{
	MathLib::Vec3d radii{ 1.0, 1.0, 1.0 }; // Radii along x, y and z axes.
	MathLib::Vec3d position{}; // Position of the center of the elipsoid.
	MathLib::Vec3d rotation{}; // Rotation angles around x, y and z axes.
	MathLib::Vec3d scale{ 1.0, 1.0, 1.0 };

	static MathLib::Vec3d color; // Elipsoid color.

	MathLib::Mat4d invertedTransformMatrix;
	bool needsUpdate = true;

	void UpdateInvertedTransformMatrix();
};

enum class InteractionMode 
{
	None,
	Rotating,
	Translating,
};

class EllipsoidApplication : public WindowApplication
{
public:
	/**
	 * @brief Creates application instance.
	 *
	 * Creates new application instance with a single window of given width, height and title.
	 *
	 * @param [in] hInstance Application instance handle (passed to WinMain by the system).
	 * @param [in] wndWidth Desired window width.
	 * @param [in] wndHeight Desired window height.
	 * @param [in] wndTitle Desired window title.
	 */
	explicit EllipsoidApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"Elipsoid");

	/**
	 * @brief Destroys the window and frees application resources.
	 */
	virtual ~EllipsoidApplication() = default;

protected:

	/**
	 * @brief Handles system messages received by the window.
	 *
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override;

	/**
	 * @brief Main loop of the application.
	 *
	 * This funtion is called by Run() to handle program's main loop.
	 *
	 * @return Application exit code.
	 */
	int MainLoop() override;
private:
	int minStep = 16;
	int step = minStep;
	Ellipsoid ellipsoid;
	InteractionMode interactionMode = InteractionMode::None;
};

