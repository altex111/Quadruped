#include "window.h"

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
	switch (msg)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProc(hwnd, msg, wparam, lparam);
}

namespace gfx
{
	void Window::RegisterWindowClass()
	{
		WNDCLASSEX wc{};
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc = WndProc;
		wc.hInstance = GetModuleHandle(NULL);
		wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wc.hIconSm = wc.hIcon;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
		wc.lpszClassName = L"Quadruped";
		wc.cbSize = sizeof(WNDCLASSEX);
		RegisterClassEx(&wc);
	}
	void Window::CreateAppWindow()
	{
		RECT rect{};
		rect.right = width;
		rect.bottom = height;
		AdjustWindowRectEx(&rect, WS_EX_OVERLAPPEDWINDOW, FALSE, WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX);
		m_appWindow = CreateWindowEx(WS_EX_OVERLAPPEDWINDOW, L"Quadruped", L"Quadruped", WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
			(GetSystemMetrics(SM_CXSCREEN) - (rect.right - rect.left)) / 2, (GetSystemMetrics(SM_CYSCREEN) - (rect.bottom - rect.top)) / 2,
			rect.right - rect.left, rect.bottom - rect.top, NULL, NULL, GetModuleHandle(NULL), NULL);
		ShowWindow(m_appWindow, SW_SHOW);
		UpdateWindow(m_appWindow);
	}
	void Window::CreateGfxWindow()
	{
		m_gfxWindow = CreateWindow(L"static", L"", WS_CHILD | WS_VISIBLE,
			0, 0, width - 250, height, m_appWindow, NULL, NULL, NULL);
		m_gfx.Init(m_gfxWindow);
	}
	void Window::CreateLabels()
	{
		m_label[0] = CreateWindow(L"static", L"", WS_VISIBLE | WS_CHILD, width - 240, 100, 200, 24, m_appWindow, NULL, NULL, NULL);
		m_label[1] = CreateWindow(L"static", L"", WS_VISIBLE | WS_CHILD, width - 240, 130, 200, 24, m_appWindow, NULL, NULL, NULL);
		m_label[2] = CreateWindow(L"static", L"", WS_VISIBLE | WS_CHILD, width - 240, 160, 200, 24, m_appWindow, NULL, NULL, NULL);
	}
	Window::Window() :
		m_appWindow(NULL),
		m_gfxWindow(NULL),
		width(1280),
		height(720)
	{
		RegisterWindowClass();
		CreateAppWindow();
		CreateGfxWindow();
		CreateLabels();
	}
	void Window::setLabelText(LPCWSTR text, int index)
	{
		SetWindowText(m_label[index], text);
	}
}