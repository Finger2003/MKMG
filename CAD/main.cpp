#include "pch.h"
#include "CadApplication.h"
#include "exceptions.h"

using namespace std;

int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
	if(FAILED(CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE)))
		return EXIT_FAILURE;

	auto exitCode = EXIT_FAILURE;
	try
	{
		CadApplication app(hInstance);
		exitCode = app.Run(nCmdShow);
	}
	catch (Exception& e)
	{
		MessageBoxW(nullptr, e.getMessage().c_str(), L"Error", MB_OK);
		exitCode = e.getExitCode();
	}
	catch (exception& e)
	{
		string s(e.what());
		MessageBoxW(nullptr, wstring(s.begin(), s.end()).c_str(), L"Error", MB_OK);
	}
	catch (const char* str)
	{
		string s(str);
		MessageBoxW(nullptr, wstring(s.begin(), s.end()).c_str(), L"Error", MB_OK);
	}
	catch (const wchar_t* str)
	{
		MessageBoxW(nullptr, str, L"Error", MB_OK);
	}
	catch (...)
	{
		MessageBoxW(nullptr, L"An unknown error has occurred.", L"Error", MB_OK);
	}
	CoUninitialize();
	return exitCode;
}