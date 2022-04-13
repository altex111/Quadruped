#include "quad_scene.h"
#include <chrono>


INT WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR szCmdLine, INT nCmdShow)
{
	gfx::Window window;
	quad::Scene scene(window);
	if (!scene.Start())
		return 0;

	MSG msg{};
	auto prevTime = std::chrono::steady_clock::now();
	while (msg.message != WM_QUIT)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			scene.MessageHandler(msg);
		}
		else
		{
			auto currentTime = std::chrono::steady_clock::now();
			scene.Frame(std::chrono::duration<float>(currentTime - prevTime).count());
			prevTime = currentTime;
		}
	}
	return (INT)msg.wParam;
}