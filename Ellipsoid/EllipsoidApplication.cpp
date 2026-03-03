#include "pch.h"
#include "EllipsoidApplication.h"

using namespace MathLib;

Vec3d Ellipsoid::color = MathLib::Vec3d(1.0, 1.0, 0.0); // Yellow color for the elipsoid.

void Ellipsoid::UpdateInvertedTransformMatrix()
{
	if (!needsUpdate)
		return;

	invertedTransformMatrix = Mat4d::CreateInverseTRS(position, rotation, scale);
	needsUpdate = false;
}


EllipsoidApplication::EllipsoidApplication(HINSTANCE hInstance, int wndWidth, int wndHeight, std::wstring wndTitle)
	: WindowApplication(hInstance, wndWidth, wndHeight, wndTitle)
{}

bool EllipsoidApplication::ProcessMessage(WindowMessage & msg)
{
	int xPos = (int)(short)LOWORD(msg.lParam);
	int yPos = (int)(short)HIWORD(msg.lParam);

	//switch (msg.message)
	//{
	//	case

	return false;
}

int EllipsoidApplication::MainLoop()
{
	return 0;
}


