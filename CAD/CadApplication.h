#pragma once
#include "DxApplication.h"
#include "../MathLib/Mat4f.h"

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

	void InitImGui();

	virtual ~CadApplication();

protected:
	void Render() override; // Renders the scene to the window.

	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> m_depthBuffer;
	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_vertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pixelShader;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> m_layout;

	MathLib::Mat4f viewProjMatrix;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbMVP;
};

struct VertexPosition
{
	float x, y, z;
};

struct Torus
{
	float majorRadius = 0.5f;
	float minorRadius = 0.2f;
	int majorSegments = 20;
	int minorSegments = 10;
	std::vector<VertexPosition> vertices;
	std::vector<unsigned int> indices;
	void GenerateMesh();

	static constexpr int cMinMajorSegments = 3;
	static constexpr int cMaxMajorSegments = 1000;
	static constexpr int cMinMinorSegments = 3;
	static constexpr int cMaxMinorSegments = 1000;

private:
	void GenerateVertices();
	void GenerateIndices();
};