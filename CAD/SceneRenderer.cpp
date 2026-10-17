#include "pch.h"
#include "SceneRenderer.h"
#include "Cursor3D.h"
#include "SceneObject.h"
#include "Torus.h"
#include "Point.h"
#include "BezierCurve.h"
#include "BSplineCurve.h"
#include "InterpolatingCurve.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
#include "GregoryPatch.h"
#include "SurfaceBuilder.h"
#include "Intersection.h"
#include "Camera.h"
#include "MillingSimulator.h"

using namespace MathLib;

SceneRenderer::SceneRenderer(const DxDevice& device) : m_device(device)
{
	const auto vsByteCode = DxDevice::LoadByteCode(L"VertexShader.cso");
	const auto psByteCode = DxDevice::LoadByteCode(L"PixelShader.cso");
	const auto pointVsByteCode = DxDevice::LoadByteCode(L"PointVS.cso");
	const auto pointPsByteCode = DxDevice::LoadByteCode(L"PointPS.cso");
	const auto pointGsByteCode = DxDevice::LoadByteCode(L"PointGS.cso");
	const auto bezierVsByteCode = DxDevice::LoadByteCode(L"BezierVS.cso");
	const auto bezierGsByteCode = DxDevice::LoadByteCode(L"BezierGS.cso");
	const auto surfaceVsByteCode = DxDevice::LoadByteCode(L"SurfaceVS.cso");
	const auto surfaceHsByteCode = DxDevice::LoadByteCode(L"SurfaceHS.cso");
	const auto surfaceDsByteCode = DxDevice::LoadByteCode(L"SurfaceDS.cso");
	const auto gregoryVsByteCode = DxDevice::LoadByteCode(L"GregoryVS.cso");
	const auto gregoryHsByteCode = DxDevice::LoadByteCode(L"GregoryHS.cso");
	const auto gregoryDsByteCode = DxDevice::LoadByteCode(L"GregoryDS.cso");
	const auto torusVSByteCode = DxDevice::LoadByteCode(L"TorusVS.cso");
	const auto trimPSByteCode = DxDevice::LoadByteCode(L"TrimPS.cso");

	m_vertexShader = device.CreateVertexShader(vsByteCode);
	m_pixelShader = device.CreatePixelShader(psByteCode);
	m_pointVertexShader = device.CreateVertexShader(pointVsByteCode);
	m_pointPixelShader = device.CreatePixelShader(pointPsByteCode);
	m_pointGeometryShader = device.CreateGeometryShader(pointGsByteCode);
	m_bezierVertexShader = device.CreateVertexShader(bezierVsByteCode);
	m_bezierGeometryShader = device.CreateGeometryShader(bezierGsByteCode);
	m_surfaceVertexShader = device.CreateVertexShader(surfaceVsByteCode);
	m_surfaceDomainShader = device.CreateDomainShader(surfaceDsByteCode);
	m_surfaceHullShader = device.CreateHullShader(surfaceHsByteCode);
	m_gregoryVertexShader = device.CreateVertexShader(gregoryVsByteCode);
	m_gregoryDomainShader = device.CreateDomainShader(gregoryDsByteCode);
	m_gregoryHullShader = device.CreateHullShader(gregoryHsByteCode);
	m_torusVertexShader = device.CreateVertexShader(torusVSByteCode);
	m_trimPixelShader = device.CreatePixelShader(trimPSByteCode);


	std::vector<D3D11_INPUT_ELEMENT_DESC> inputElements = {
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	m_layout = device.CreateInputLayout(inputElements, vsByteCode);

	std::vector<D3D11_INPUT_ELEMENT_DESC> uvInputElements = {
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};
	m_layoutUV = device.CreateInputLayout(uvInputElements, torusVSByteCode);

	m_cbPerPass = device.CreateConstantBuffer<PerPassBuffer>();
	m_cbPerObject = device.CreateConstantBuffer<PerObjectBuffer>();
	InitStereoBlendStates();

	D3D11_TEXTURE2D_DESC texDesc = {};
	texDesc.Width = 1;
	texDesc.Height = 1;
	texDesc.MipLevels = 1;
	texDesc.ArraySize = 1;
	texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	texDesc.SampleDesc.Count = 1;
	texDesc.Usage = D3D11_USAGE_IMMUTABLE;
	texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

	uint32_t whitePixel = 0xFFFFFFFF;
	D3D11_SUBRESOURCE_DATA initData = { &whitePixel, sizeof(uint32_t), 0 };
	Microsoft::WRL::ComPtr<ID3D11Texture2D> dummyTex;
	device.get()->CreateTexture2D(&texDesc, &initData, dummyTex.GetAddressOf());
	device.get()->CreateShaderResourceView(dummyTex.Get(), nullptr, m_dummyTrimTextureSRV.GetAddressOf());

	D3D11_SAMPLER_DESC sampDesc = {};
	sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_CLAMP;
	sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
	sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
	sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	device.get()->CreateSamplerState(&sampDesc, m_trimSampler.GetAddressOf());

	Cursor3D::InitSharedGeometry(device);
}

void SceneRenderer::DrawScene(const Camera& camera, const std::vector<std::shared_ptr<SceneObject>>& sceneObjects, 
	bool enableStereo, const MathLib::Vec4f& leftEyeColor, const MathLib::Vec4f& rightEyeColor, float eyeSeparation, float focalLength, 
	const float3& cursorPosition, std::optional<float3> selectionCenter, bool showBernsteinPoints, SIZE renderSize, ID3D11DepthStencilView* depthBuffer, const SurfaceBuilder* surfaceBuilder, const MillingSimulator* millingSimulator)
{
	auto& context = m_device.getContext();

	if (!enableStereo)
	{
		context->OMSetBlendState(m_blendStateDefault.Get(), nullptr, 0xFFFFFFFF);
		context->ClearDepthStencilView(depthBuffer, D3D11_CLEAR_DEPTH, 1.0f, 0);
		SyncPerPassBuffer(camera, renderSize);
		DrawScenePass(sceneObjects, cursorPosition, selectionCenter, showBernsteinPoints, surfaceBuilder, millingSimulator);
	}
	else
	{
		context->OMSetBlendState(m_blendStateAnaglyph.Get(), nullptr, 0xFFFFFFFF);

		// Left eye
		context->ClearDepthStencilView(depthBuffer, D3D11_CLEAR_DEPTH, 1.0f, 0);
		SetupStereoCamera(camera, true, leftEyeColor, eyeSeparation, focalLength, renderSize);
		DrawScenePass(sceneObjects, cursorPosition, selectionCenter, showBernsteinPoints, surfaceBuilder, millingSimulator);

		// Right eye
		context->ClearDepthStencilView(depthBuffer, D3D11_CLEAR_DEPTH, 1.0f, 0);
		SetupStereoCamera(camera, false, rightEyeColor, eyeSeparation, focalLength, renderSize);
		DrawScenePass(sceneObjects, cursorPosition, selectionCenter, showBernsteinPoints, surfaceBuilder, millingSimulator);

		context->OMSetBlendState(m_blendStateDefault.Get(), nullptr, 0xFFFFFFFF);
	}
}

void SceneRenderer::RenderTrimTextures(Intersection* intersection)
{
	auto& context = m_device.getContext();

	static Microsoft::WRL::ComPtr<ID3D11DepthStencilState> dsMarkStencil, dsApplyMask;
	static Microsoft::WRL::ComPtr<ID3D11BlendState> bsNoColor;
	if (!dsMarkStencil)
	{
		D3D11_DEPTH_STENCIL_DESC desc = {};
		desc.DepthEnable = FALSE;
		desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		desc.DepthFunc = D3D11_COMPARISON_ALWAYS;

		desc.StencilEnable = TRUE;
		desc.StencilReadMask = 0xFF;
		desc.StencilWriteMask = 0xFF;

		D3D11_DEPTH_STENCILOP_DESC stencilOp = {};
		stencilOp.StencilFailOp = D3D11_STENCIL_OP_KEEP;
		stencilOp.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
		stencilOp.StencilPassOp = D3D11_STENCIL_OP_INVERT;
		stencilOp.StencilFunc = D3D11_COMPARISON_ALWAYS;

		desc.FrontFace = stencilOp;
		desc.BackFace = stencilOp;

		HRESULT hr = m_device.get()->CreateDepthStencilState(
			&desc,
			dsMarkStencil.GetAddressOf()
		);
		assert(SUCCEEDED(hr));

		D3D11_BLEND_DESC bDesc = {};
		bDesc.RenderTarget[0].RenderTargetWriteMask = 0; // Write no color
		hr = m_device.get()->CreateBlendState(
			&bDesc,
			bsNoColor.GetAddressOf()
		);
		assert(SUCCEEDED(hr));

		stencilOp.StencilPassOp = D3D11_STENCIL_OP_KEEP;
		stencilOp.StencilFunc = D3D11_COMPARISON_NOT_EQUAL;

		desc.FrontFace = stencilOp;
		desc.BackFace = stencilOp;

		hr = m_device.get()->CreateDepthStencilState(
			&desc,
			dsApplyMask.GetAddressOf()
		);
		assert(SUCCEEDED(hr));
	}

	static Microsoft::WRL::ComPtr<ID3D11RasterizerState> rsCullNone;
	if (!rsCullNone)
	{
		D3D11_RASTERIZER_DESC rDesc = {};
		rDesc.FillMode = D3D11_FILL_SOLID;
		rDesc.CullMode = D3D11_CULL_NONE; // Prevent arbitrary winding deletion
		rDesc.FrontCounterClockwise = FALSE;
		rDesc.DepthClipEnable = TRUE;
		m_device.get()->CreateRasterizerState(&rDesc, rsCullNone.GetAddressOf());
	}

	static Microsoft::WRL::ComPtr<ID3D11Buffer> s_fullscreenQuad;
	if (!s_fullscreenQuad)
	{
		std::vector<VertexPosition> quadVerts = {
			{ -1.0f, -1.0f, 0.0f }, // Bottom-Left
			{ -1.0f,  1.0f, 0.0f }, // Top-Left
			{ 1.0f, -1.0f, 0.0f }, // Bottom-Right
			{ 1.0f,  1.0f, 0.0f }  // Top-Right
		};
		s_fullscreenQuad = m_device.CreateVertexBuffer(quadVerts);
	}

	Microsoft::WRL::ComPtr<ID3D11RenderTargetView> oldRTV;
	Microsoft::WRL::ComPtr<ID3D11DepthStencilView> oldDSV;
	context->OMGetRenderTargets(1, oldRTV.GetAddressOf(), oldDSV.GetAddressOf());

	UINT numViewports = 1;
	D3D11_VIEWPORT oldViewport;
	context->RSGetViewports(&numViewports, &oldViewport);

	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);


	context->GSSetShader(nullptr, nullptr, 0);
	context->HSSetShader(nullptr, nullptr, 0);
	context->DSSetShader(nullptr, nullptr, 0);

	context->IASetInputLayout(m_layout.Get());
	ID3D11Buffer* cBuffers[] = { m_cbPerPass.Get(), m_cbPerObject.Get() };
	context->VSSetConstantBuffers(0, 2, cBuffers);
	context->PSSetConstantBuffers(0, 2, cBuffers);

	PerPassBuffer passData;
	passData.stereoTint = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
	passData.viewProj = Mat4f::Identity();
	passData.aspectRatio = 1.0f;
	passData.renderSize[0] = 512.0f;
	passData.renderSize[1] = 512.0f;

	m_device.UpdateBuffer(m_cbPerPass, passData);

	auto renderMask = [&](TrimTexture& tex, UINT triCount, ID3D11Buffer* triBuffer, UINT lineCount,
		ID3D11Buffer* lineBuffer, bool reverse) {

			D3D11_VIEWPORT vp = { 0.0f, 0.0f, static_cast<float>(tex.width), static_cast<float>(tex.height), 0.0f, 1.0f };
			context->RSSetViewports(1, &vp);

			// Clear Target: White (keep) or Black (discard) based on reverse flag
			const float clearColor[4] = { reverse ? 0.0f : 1.0f, reverse ? 0.0f : 1.0f, reverse ? 0.0f : 1.0f, 1.0f };
			context->OMSetRenderTargets(1, tex.rtv.GetAddressOf(), tex.dsv.Get());
			context->ClearRenderTargetView(tex.rtv.Get(), clearColor);
			context->ClearDepthStencilView(tex.dsv.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

			MathLib::Vec4f drawColor = reverse
				? MathLib::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)
				: MathLib::Vec4f(0.0f, 0.0f, 0.0f, 1.0f);

			//if (count == 0) return;
			if (triCount == 0 || triBuffer == nullptr)
			{
				if (lineCount > 0 && lineBuffer != nullptr)
				{
					PerObjectBuffer objData{};
					objData.model = Mat4f::Identity();
					objData.color = drawColor;
					m_device.UpdateBuffer(m_cbPerObject, objData);

					context->RSSetState(nullptr);
					context->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF);
					context->OMSetDepthStencilState(nullptr, 0);
					context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

					UINT stride = sizeof(VertexPosition);
					UINT offset = 0;
					context->IASetVertexBuffers(0, 1, &lineBuffer, &stride, &offset);
					context->Draw(lineCount, 0);
				}

				return;
			}

			PerObjectBuffer objData{};
			objData.model = Mat4f::Identity();
			objData.color = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			// PASS 1: Draw invisible triangles to mark the Stencil Buffer
			context->RSSetState(rsCullNone.Get());
			context->OMSetBlendState(bsNoColor.Get(), nullptr, 0xFFFFFFFF);
			context->OMSetDepthStencilState(dsMarkStencil.Get(), 0);
			context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, &triBuffer, &stride, &offset);
			context->Draw(triCount, 0);

			// PASS 2: Draw Fullscreen Quad where Stencil is 1
			objData.color = drawColor;
			m_device.UpdateBuffer(m_cbPerObject, objData);

			context->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF); // Restore Color
			context->OMSetDepthStencilState(dsApplyMask.Get(), 0); // Test Stencil == 1
			context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);

			context->IASetVertexBuffers(0, 1, s_fullscreenQuad.GetAddressOf(), &stride, &offset);
			context->Draw(4, 0);
		};


	renderMask(intersection->m_trimTexture1, intersection->m_trimPolygonCount1, intersection->m_trimPolygonBuffer1.Get(), intersection->m_uvLineCount1, intersection->m_uvLinesBuffer1.Get(), intersection->m_reverseTrimS1);
	renderMask(intersection->m_trimTexture2, intersection->m_trimPolygonCount2, intersection->m_trimPolygonBuffer2.Get(), intersection->m_uvLineCount2, intersection->m_uvLinesBuffer2.Get(), intersection->m_reverseTrimS2);

	context->OMSetRenderTargets(1, oldRTV.GetAddressOf(), oldDSV.Get());
	context->RSSetViewports(1, &oldViewport);

	context->OMSetDepthStencilState(nullptr, 0);
	context->OMSetBlendState(nullptr, nullptr, 0xFFFFFFFF);
	context->RSSetState(nullptr);
}

void SceneRenderer::InitStereoBlendStates()
{
	m_blendStateDefault = m_device.CreateBlendState();
	m_blendStateAnaglyph = m_device.CreateBlendState(BlendDescription::MaxBlendDescription());
}

void SceneRenderer::SetupStereoCamera(const Camera& camera, bool isLeftEye, const MathLib::Vec4f& eyeTint, float eyeSeparation, float focalLength, SIZE renderSize)
{
	float E = eyeSeparation * 0.5f;
	float shiftMultiplier = isLeftEye ? 1.0f : -1.0f;
	float E_shift = E * shiftMultiplier;

	Mat4f stereoView = Mat4f::Translation(E_shift, 0.0f, 0.0f) * camera.GetViewMatrix();
	float aspect = camera.GetAspectRatio();
	float fov = camera.GetFovY();
	float n = camera.GetNearPlane();
	float f = camera.GetFarPlane();

	float top = n * std::tan(fov / 2.0f);
	float bottom = -top;

	float frustumShift = E_shift * (n / focalLength);
	float width = aspect * top;

	float left = -width + frustumShift;
	float right = width + frustumShift;

	Mat4f stereoProj = Mat4f::Frustum(left, right, bottom, top, n, f);

	PerPassBuffer perPassData;
	perPassData.viewProj = stereoProj * stereoView;
	perPassData.aspectRatio = aspect;
	perPassData.renderSize[0] = static_cast<float>(renderSize.cx);
	perPassData.renderSize[1] = static_cast<float>(renderSize.cy);
	perPassData.stereoTint = eyeTint;
	m_device.UpdateBuffer(m_cbPerPass, perPassData);
}

void SceneRenderer::SyncPerPassBuffer(const Camera& camera, SIZE renderSize)
{
	if (m_cbPerPass)
	{
		PerPassBuffer perPassData;
		perPassData.viewProj = camera.GetProjViewMatrix();
		perPassData.aspectRatio = camera.GetAspectRatio();
		perPassData.renderSize[0] = static_cast<float>(renderSize.cx);
		perPassData.renderSize[1] = static_cast<float>(renderSize.cy);
		perPassData.stereoTint = { 1.0f, 1.0f, 1.0f, 1.0f };
		m_device.UpdateBuffer(m_cbPerPass, perPassData);
	}
}

void SceneRenderer::DrawScenePass(const std::vector<std::shared_ptr<SceneObject>>& sceneObjects, const float3& cursorPosition, const std::optional<float3>& selectionCenter, 
	bool showBernsteinPoints, const SurfaceBuilder* surfaceBuilder, const MillingSimulator* millingSimulator)
{
	auto& context = m_device.getContext();

	// Global buffers and samplers
	ID3D11Buffer* buffers[] = { m_cbPerPass.Get(), m_cbPerObject.Get() };
	context->VSSetConstantBuffers(0, 2, buffers);
	context->HSSetConstantBuffers(0, 2, buffers);
	context->DSSetConstantBuffers(0, 2, buffers);
	context->GSSetConstantBuffers(0, 2, buffers);
	context->PSSetConstantBuffers(0, 2, buffers);
	context->PSSetSamplers(0, 1, m_trimSampler.GetAddressOf());

	context->IASetInputLayout(m_layout.Get());
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);

	// Toruses and Cursors
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	context->PSSetShader(m_trimPixelShader.Get(), nullptr, 0);
	DrawToruses(sceneObjects);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);

	DrawCursors(cursorPosition, selectionCenter);

	// LineStrip Elements
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP);
	DrawPolylines(sceneObjects, showBernsteinPoints);
	DrawLinearIntersections(sceneObjects);
	DrawMillingPath(millingSimulator);

	// LineList Elements
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
	DrawSurfacesPolylines(sceneObjects);
	DrawGregoryPatchesTangents(sceneObjects);

	// Bezier Curves
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST_ADJ);
	context->VSSetShader(m_bezierVertexShader.Get(), nullptr, 0);
	context->GSSetShader(m_bezierGeometryShader.Get(), nullptr, 0);
	DrawBezierCurves(sceneObjects);
	DrawBezierIntersections(sceneObjects);

	// Points
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
	context->VSSetShader(m_vertexShader.Get(), nullptr, 0);
	context->GSSetShader(m_pointGeometryShader.Get(), nullptr, 0);
	context->PSSetShader(m_pointPixelShader.Get(), nullptr, 0);
	DrawPoints(sceneObjects, surfaceBuilder);
	DrawVirtualBernsteinPoints(sceneObjects, showBernsteinPoints);

	context->GSSetShader(nullptr, nullptr, 0);

	// Surfaces
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_16_CONTROL_POINT_PATCHLIST);

	context->VSSetShader(m_surfaceVertexShader.Get(), nullptr, 0);
	context->HSSetShader(m_surfaceHullShader.Get(), nullptr, 0);
	context->DSSetShader(m_surfaceDomainShader.Get(), nullptr, 0);
	context->PSSetShader(m_trimPixelShader.Get(), nullptr, 0);

	DrawSurfaces(sceneObjects, surfaceBuilder);

	// Gregory Patches
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_20_CONTROL_POINT_PATCHLIST);
	context->VSSetShader(m_gregoryVertexShader.Get(), nullptr, 0);
	context->HSSetShader(m_gregoryHullShader.Get(), nullptr, 0);
	context->DSSetShader(m_gregoryDomainShader.Get(), nullptr, 0);
	context->PSSetShader(m_pixelShader.Get(), nullptr, 0);

	DrawGregoryPatches(sceneObjects);

	context->HSSetShader(nullptr, nullptr, 0);
	context->DSSetShader(nullptr, nullptr, 0);
}

void SceneRenderer::DrawCursors(const float3& cursorPosition, const std::optional<float3>& selectionCenter)
{
	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	m_device.getContext()->IASetVertexBuffers(0, 1, Cursor3D::GetStaticVertexBuffer().GetAddressOf(), &stride, &offset);
	DrawCursor(cursorPosition, 0.1f);
	if (selectionCenter)
		DrawCursor(*selectionCenter, 0.06f);
}

void SceneRenderer::DrawCursor(const float3& position, float scale)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	Mat4f translation = Mat4f::Translation(position.x, position.y, position.z);
	Mat4f scaling = Mat4f::Scaling(scale);

	// X axis - Red
	objData.model = translation * scaling;
	objData.color = { 1.0f, 0.0f, 0.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);

	// Y axis - Green (Rotate X arrow 90 degrees around Z axis)
	objData.model = translation * Mat4f::RotationZ(std::numbers::pi_v<float> / 2.0f) * scaling;
	objData.color = { 0.0f, 1.0f, 0.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);

	// Z axis - Blue (Rotate X arrow -90 degrees around Y axis)
	objData.model = translation * Mat4f::RotationY(-std::numbers::pi_v<float> / 2.0f) * scaling;
	objData.color = { 0.0f, 0.0f, 1.0f, 1.0f };
	m_device.UpdateBuffer(m_cbPerObject, objData);
	context->Draw(Cursor3D::VertexCount, 0);
}

void SceneRenderer::DrawToruses(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();
	context->IASetInputLayout(m_layoutUV.Get());
	context->VSSetShader(m_torusVertexShader.Get(), nullptr, 0);

	for (auto& obj : objects)
	{
		if (auto torus = obj->As<Torus>())
		{
			torus->UpdateMesh(m_device);

			ID3D11ShaderResourceView* srvToBind = torus->m_trimTextureSRV ? torus->m_trimTextureSRV.Get() : m_dummyTrimTextureSRV.Get();
			context->PSSetShaderResources(0, 1, &srvToBind);

			PerObjectBuffer objData;
			objData.model = torus->m_modelMatrix;
			objData.color = torus->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			UINT stride = sizeof(VertexPositionUV);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, torus->GetVertexBuffer().GetAddressOf(), &stride, &offset);
			context->IASetIndexBuffer(torus->GetIndexBuffer().Get(), DXGI_FORMAT_R32_UINT, 0);
			context->DrawIndexed(static_cast<UINT>(torus->indices.size()), 0, 0);
		}
	}
}

void SceneRenderer::DrawPoints(const std::vector<std::shared_ptr<SceneObject>>& objects, const SurfaceBuilder* surfaceBuilder)
{
	auto& context = m_device.getContext();
	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, Point::GetSharedVertexBuffer().GetAddressOf(), &stride, &offset);
	for (auto& obj : objects)
	{
		if (auto point = obj->As<Point>())
		{
			PerObjectBuffer objData;
			objData.model = point->GetModelMatrix();
			objData.color = point->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->Draw(1, 0);
		}
	}

	if (surfaceBuilder)
	{
		for (auto& point : surfaceBuilder->rawPoints)
		{
			PerObjectBuffer objData;
			objData.model = Mat4f::Translation(point.x, point.y, point.z);
			objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->Draw(1, 0);
		}
	}
}

void SceneRenderer::DrawPolylines(const std::vector<std::shared_ptr<SceneObject>>& objects, bool showBernsteinPoints)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.7f, 0.7f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : objects)
	{
		if (auto curve = obj->As<Curve>())
		{
			curve->UpdatePolyline(m_device);
			if (curve->selected)
			{
				ID3D11Buffer** bufferToDraw = curve->m_lineVertexBuffer.GetAddressOf();
				UINT countToDraw = curve->m_lineVertexCount;

				if (showBernsteinPoints)
				{
					if (auto bsplineCurve = curve->As<BSplineCurve>())
					{
						bufferToDraw = bsplineCurve->m_bernsteinVertexBuffer.GetAddressOf();
						countToDraw = bsplineCurve->m_bernsteinVertexCount;
					}
				}

				if (countToDraw > 0)
				{
					UINT stride = sizeof(VertexPosition);
					UINT offset = 0;
					context->IASetVertexBuffers(0, 1, bufferToDraw, &stride, &offset);
					context->Draw(countToDraw, 0);
				}
			}
		}
	}
}

void SceneRenderer::DrawBezierCurves(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();

	for (auto& obj : objects)
	{
		if (auto curve = obj->As<Curve>())
		{
			objData.color = curve->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			m_device.UpdateBuffer(m_cbPerObject, objData);

			if (curve->m_curveVertexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, curve->m_curveVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(curve->m_curveVertexCount, 0);
			}
		}
	}
}

void SceneRenderer::DrawVirtualBernsteinPoints(const std::vector<std::shared_ptr<SceneObject>>& objects, bool showBernsteinPoints)
{
	if (!showBernsteinPoints) 
		return;

	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : objects)
	{
		if (!obj->selected)
			continue;

		if (auto bspline = obj->As<BSplineCurve>())
		{
			if (bspline->m_bernsteinVertexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, bspline->m_bernsteinVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(bspline->m_bernsteinVertexCount, 0);
			}
		}
	}
}

void SceneRenderer::DrawSurfaces(const std::vector<std::shared_ptr<SceneObject>>& objects, const SurfaceBuilder* surfaceBuilder)
{
	auto& context = m_device.getContext();

	context->IASetInputLayout(m_layoutUV.Get());
	if (surfaceBuilder && surfaceBuilder->m_patchIndexCount > 0)// && m_previewSurface)
	{
		UINT stride = sizeof(VertexPositionUV);
		UINT offset = 0;
		context->IASetVertexBuffers(0, 1, surfaceBuilder->m_patchVertexBuffer.GetAddressOf(), &stride, &offset);
		context->IASetIndexBuffer(surfaceBuilder->m_patchIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
		PerObjectBuffer objData;
		objData.color = Vec4f(0.0f, 0.5f, 1.0f, 1.0f);
		auto updateAndDraw = [&](float xParam, float yParam) {
			objData.surfaceParams =
			{
				xParam,
				yParam,
				25.0f,
				0.0f
			};
			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->DrawIndexed(surfaceBuilder->m_patchIndexCount, 0, 0);
			};

		updateAndDraw(0.0f, surfaceBuilder->linesPerSegmentU);
		updateAndDraw(1.0f, surfaceBuilder->linesPerSegmentV);
	}

	for (auto& obj : objects)
	{
		if (auto surface = obj->As<Surface>())
			DrawSurface(context, surface, surface->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f));
	}

	context->PSSetShaderResources(0, 1, m_dummyTrimTextureSRV.GetAddressOf());
	context->IASetInputLayout(m_layout.Get());
}

void SceneRenderer::DrawSurface(const Microsoft::WRL::ComPtr<ID3D11DeviceContext>& context, Surface* surface, MathLib::Vec4f color)
{
	surface->UpdateVertices(m_device);
	if (surface->m_patchIndexCount > 0)
	{
		UINT stride = sizeof(VertexPositionUV);
		UINT offset = 0;
		context->IASetVertexBuffers(0, 1, surface->m_patchVertexBuffer.GetAddressOf(), &stride, &offset);
		context->IASetIndexBuffer(surface->m_patchIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
		ID3D11ShaderResourceView* srvToBind = surface->m_trimTextureSRV ? surface->m_trimTextureSRV.Get() : m_dummyTrimTextureSRV.Get();
		context->PSSetShaderResources(0, 1, &srvToBind);
		PerObjectBuffer objData;
		objData.color = color;

		auto updateAndDraw = [&](Surface* surf, float xParam, float yParam) {
			objData.surfaceParams =
			{
				xParam,
				yParam,
				static_cast<float>(surf->m_smoothness),
				0.0f
			};

			m_device.UpdateBuffer(m_cbPerObject, objData);
			context->DrawIndexed(surf->m_patchIndexCount, 0, 0);
			};

		updateAndDraw(surface, 0.0f, static_cast<float>(surface->m_linesPerSegmentU));
		updateAndDraw(surface, 1.0f, static_cast<float>(surface->m_linesPerSegmentV));
	}
}

void SceneRenderer::DrawSurfacesPolylines(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.7f, 0.7f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : objects)
	{
		if (auto surface = obj->As<Surface>())
		{
			surface->UpdateVertices(m_device);
			if (surface->selected && surface->m_polylineIndexCount > 0)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, surface->m_polylineVertexBuffer.GetAddressOf(), &stride, &offset);
				context->IASetIndexBuffer(surface->m_polylineIndexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
				context->DrawIndexed(surface->m_polylineIndexCount, 0, 0);
			}
		}
	}
}

void SceneRenderer::DrawGregoryPatches(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	for (auto& obj : objects)
	{
		if (obj->type == ObjectType::GregoryPatch)
		{
			auto gregoryPatch = static_cast<GregoryPatch*>(obj.get());
			gregoryPatch->UpdateVertices(m_device);

			UINT stride = sizeof(VertexPosition);
			UINT offset = 0;
			context->IASetVertexBuffers(0, 1, gregoryPatch->m_vertexBuffer.GetAddressOf(), &stride, &offset);

			PerObjectBuffer objData;
			objData.color = gregoryPatch->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);

			auto updateAndDraw = [&](float xParam, float yParam) {
				objData.surfaceParams =
				{
					xParam,
					yParam,
					static_cast<float>(gregoryPatch->m_smoothness),
					0.0f
				};
				m_device.UpdateBuffer(m_cbPerObject, objData);
				context->Draw(gregoryPatch->m_vertexCount, 0);
				};
			updateAndDraw(0.0f, static_cast<float>(gregoryPatch->m_linesPerSegmentU));
			updateAndDraw(1.0f, static_cast<float>(gregoryPatch->m_linesPerSegmentV));
		}
	}
}

void SceneRenderer::DrawGregoryPatchesTangents(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.7f, 0.1f, 0.1f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	for (auto& obj : objects)
	{
		if (auto patch = obj->As<GregoryPatch>())
		{
			patch->UpdateVertices(m_device);
			if (patch->selected)
			{
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, patch->m_continuityVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(patch->m_continuityVertexCount, 0);
			}
		}
	}
}

void SceneRenderer::DrawLinearIntersections(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	for (const auto& obj : objects)
	{
		if (obj->type == ObjectType::LinearIntersection)
		{
			auto intersection = static_cast<LinearIntersection*>(obj.get());

			if (intersection->m_vertexCount > 0)
			{
				objData.color = intersection->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
				m_device.UpdateBuffer(m_cbPerObject, objData);

				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, intersection->m_vertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(intersection->m_vertexCount, 0);
			}
		}
	}
}

void SceneRenderer::DrawBezierIntersections(const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	auto& context = m_device.getContext();

	PerObjectBuffer objData;
	objData.model = Mat4f::Identity();
	for (const auto& obj : objects)
	{
		if (obj->type == ObjectType::BezierIntersection)
		{
			auto intersection = static_cast<BezierIntersection*>(obj.get());
			if (intersection->m_bezierVertexCount > 0)
			{
				objData.color = intersection->selected ? Vec4f(1.0f, 1.0f, 0.0f, 1.0f) : Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
				m_device.UpdateBuffer(m_cbPerObject, objData);
				UINT stride = sizeof(VertexPosition);
				UINT offset = 0;
				context->IASetVertexBuffers(0, 1, intersection->m_bezierVertexBuffer.GetAddressOf(), &stride, &offset);
				context->Draw(intersection->m_bezierVertexCount, 0);
			}
		}
	}
}

void SceneRenderer::DrawMillingPath(const MillingSimulator* millingSimulator)
{
	if (!millingSimulator || millingSimulator->getPathVertexCount() < 2)
		return;

	auto& context = m_device.getContext();

	PerObjectBuffer objData{};
	objData.model = Mat4f::Identity();
	objData.color = Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
	m_device.UpdateBuffer(m_cbPerObject, objData);

	UINT stride = sizeof(VertexPosition);
	UINT offset = 0;
	context->IASetVertexBuffers(0, 1, millingSimulator->getPathVertexBuffer().GetAddressOf(), &stride, &offset);
	context->Draw(millingSimulator->getPathVertexCount(), 0);
}