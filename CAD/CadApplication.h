#pragma once
#include "DxApplication.h"
#include "../MathLib/Mat4f.h"
#include "Torus.h"

struct PerObjectBuffer
{
	MathLib::Mat4f model;
};

struct PerPassBuffer
{
	MathLib::Mat4f viewProj;
};

class CadApplication : public DxApplication
{

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
public:
	explicit CadApplication(HINSTANCE hInstance,
		int wndWidth = Window::m_defaultWindowWidth,
		int wndHeight = Window::m_defaultWindowHeight,
		std::wstring wndTitle = L"CADApp");

	/**
	 * @brief Handles system messages received by the window.
	 *
	 * @param [in, out] msg contains message ID and its parameters.
	 * @return true if the message is processed and should not be passed
	 * to the default window procedure, false otherwise.
	 */
	bool ProcessMessage(WindowMessage& msg) override;

	virtual ~CadApplication();

	/**
	 * @brief Updates application resources that depend on the size of the window.
	 *
	 * @param [in] width New width of the window's client area.
	 * @param [in] height New height of the window's client area.
	 */
	void UpdateResources(int width, int height) override;

protected:
	void Render() override; // Renders the scene to the window.

	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> m_depthBuffer;
	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_vertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pixelShader;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> m_layout;

	MathLib::Mat4f m_viewMatrix;
	MathLib::Mat4f m_projMatrix;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerObject;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerPass;

private:
	Torus m_torus;
	void DrawMenu(int width, int height);
	void InitImGui();
};

