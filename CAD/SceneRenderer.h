#pragma once

#include "DxDevice.h"
#include "../MathLib/Vec4f.h"
#include "../MathLib/Vec3f.h"
#include "../MathLib/Mat4f.h"


class Camera;
struct SceneObject;
struct Surface;
struct Intersection;
struct SurfaceBuilder;
struct float3;
class MillingSimulator;

struct PerPointBuffer
{
	MathLib::Vec4f color;
};


struct PerObjectBuffer
{
	MathLib::Mat4f model;
	MathLib::Vec4f color;
	MathLib::Vec4f surfaceParams;
};

struct PerPassBuffer
{
	MathLib::Mat4f viewProj;
	float aspectRatio;
	float renderSize[2];
	float padding;
	MathLib::Vec4f stereoTint;
};


class SceneRenderer
{
public:
	SceneRenderer(const DxDevice& device);
	~SceneRenderer() = default;

	void DrawScene(
		const Camera& camera,
		const std::vector<std::shared_ptr<SceneObject>>& sceneObjects,
		bool enableStereo,
		const MathLib::Vec4f& leftEyeColor,
		const MathLib::Vec4f& rightEyeColor,
		float eyeSeparation,
		float focalLength,
		const float3& cursorPosition,
		std::optional<float3> selectionCenter,
		bool showBernsteinPoints,
		SIZE renderSize,
		ID3D11DepthStencilView* depthBuffer,
		const SurfaceBuilder* surfaceBuilder = nullptr,
		const MillingSimulator* millingSimulator = nullptr
		);

	void RenderTrimTextures(Intersection* intersection);

private:
	const DxDevice& m_device;

	//SIZE m_renderSize{};

	//Microsoft::WRL::ComPtr<ID3D11DepthStencilView> m_depthBuffer;
	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_vertexShader;
	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_torusVertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pixelShader;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> m_layout;
	Microsoft::WRL::ComPtr<ID3D11InputLayout> m_layoutUV;

	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_pointVertexShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pointPixelShader;
	Microsoft::WRL::ComPtr<ID3D11PixelShader> m_trimPixelShader;
	Microsoft::WRL::ComPtr<ID3D11GeometryShader> m_pointGeometryShader;
	//Microsoft::WRL::ComPtr<ID3D11InputLayout> m_pointLayout;

	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_bezierVertexShader;
	Microsoft::WRL::ComPtr<ID3D11GeometryShader> m_bezierGeometryShader;

	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_surfaceVertexShader;
	Microsoft::WRL::ComPtr<ID3D11DomainShader> m_surfaceDomainShader;
	Microsoft::WRL::ComPtr<ID3D11HullShader> m_surfaceHullShader;

	Microsoft::WRL::ComPtr<ID3D11VertexShader> m_gregoryVertexShader;
	Microsoft::WRL::ComPtr<ID3D11DomainShader> m_gregoryDomainShader;
	Microsoft::WRL::ComPtr<ID3D11HullShader> m_gregoryHullShader;


	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerObject;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_cbPerPass;

	Microsoft::WRL::ComPtr<ID3D11BlendState> m_blendStateAnaglyph;
	Microsoft::WRL::ComPtr<ID3D11BlendState> m_blendStateDefault;

	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_dummyTrimTextureSRV;
	Microsoft::WRL::ComPtr<ID3D11SamplerState> m_trimSampler;

	// Internal stereoscopy helpers
    void InitStereoBlendStates();
    void SetupStereoCamera(const Camera& camera, bool isLeftEye, const MathLib::Vec4f& eyeTint, float eyeSeparation, float focalLength, SIZE renderSize);
    void SyncPerPassBuffer(const Camera& camera, SIZE renderSize);

    // Internal rendering helpers
    void DrawScenePass(const std::vector<std::shared_ptr<SceneObject>>& sceneObjects, const float3& cursorPosition, const std::optional<float3>& selectionCenter, 
		bool showBernsteinPoints, const SurfaceBuilder* surfaceBuilder, const MillingSimulator* millingSimulator);
    
    void DrawCursors(const float3& cursorPosition, const std::optional<float3>& selectionCenter);
    void DrawCursor(const float3& position, float scale);
    void DrawToruses(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawPoints(const std::vector<std::shared_ptr<SceneObject>>& objects, const SurfaceBuilder* surfaceBuilder);
    void DrawPolylines(const std::vector<std::shared_ptr<SceneObject>>& objects, bool showBernsteinPoints);
    void DrawBezierCurves(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawVirtualBernsteinPoints(const std::vector<std::shared_ptr<SceneObject>>& objects, bool showBernsteinPoints);
    void DrawSurfaces(const std::vector<std::shared_ptr<SceneObject>>& objects, const SurfaceBuilder* surfaceBuilder);
    void DrawSurface(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context, Surface* surface, MathLib::Vec4f color);
    void DrawSurfacesPolylines(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawGregoryPatches(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawGregoryPatchesTangents(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawLinearIntersections(const std::vector<std::shared_ptr<SceneObject>>& objects);
    void DrawBezierIntersections(const std::vector<std::shared_ptr<SceneObject>>& objects);
	void DrawMillingPath(const MillingSimulator* millingSimulator);
};