#include "pch.h"
#include "Cursor3D.h"
#include "DxDevice.h"

using namespace std;
using namespace Microsoft::WRL;
using namespace MathLib;

ComPtr<ID3D11Buffer> Cursor3D::s_vertexBuffer = nullptr;

void Cursor3D::InitSharedGeometry(const DxDevice& device)
{
	if (s_vertexBuffer)
		return; // Already initialized

	vector<VertexPosition> vertices =
	{
		{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},		// Main shaft
		{1.0f, 0.0f, 0.0f}, {0.8f, 0.1f, 0.0f},		// Arrowhead XY
		{1.0f, 0.0f, 0.0f}, {0.8f, -0.1f, 0.0f},	// Arrowhead XY
		{1.0f, 0.0f, 0.0f}, {0.8f, 0.0f, 0.1f},		// Arrowhead XZ
		{1.0f, 0.0f, 0.0f}, {0.8f, 0.0f, -0.1f}		// Arrowhead XZ
	};

	s_vertexBuffer = device.CreateVertexBuffer(vertices);
}

void Cursor3D::ReleaseSharedGeometry()
{
	s_vertexBuffer.Reset();
}

//MathLib::Mat4f Cursor3D::GetTranslationMatrix() const
//{
//	return Mat4f::Translation(position.x, position.y, position.z);
//}
