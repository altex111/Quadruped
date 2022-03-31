#pragma once

#include "graphics.h"

namespace gfx
{
	class Window
	{
		HWND m_appWindow;
		HWND m_gfxWindow;
		HWND m_label[3];
		int width;
		int height;

		Graphics m_gfx;

	private:
		void RegisterWindowClass();
		void CreateAppWindow();
		void CreateGfxWindow();
		void CreateLabels();

	public:
		Window();

		inline Graphics& getGfx() { return m_gfx; }
		void setLabelText(LPCWSTR text, int index);
	};
}