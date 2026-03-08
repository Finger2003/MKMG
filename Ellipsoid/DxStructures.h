#pragma once


struct SwapChainDescription : DXGI_SWAP_CHAIN_DESC
{
	SwapChainDescription(HWND hWnd, SIZE wndSize);
};

struct Texture2DDescription : D3D11_TEXTURE2D_DESC
{
	Texture2DDescription(SIZE size);
};
