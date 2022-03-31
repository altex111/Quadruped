#include "quad_scene.h"
#include <string>

namespace quad
{
	Scene::Scene(gfx::Window& window) :
		m_window(window)
	{
	}
	bool Scene::Start()
	{
		auto& gfx = m_window.getGfx();
		auto device = gfx.getDevice();
		if (!m_renderer.Init(gfx))
			return false;

		if (!m_plain.Init(device, L"resources/plain.mdl", mth::float4(0.2f, 1.0f, 0.35f, 1.0f)))
			return false;
		m_renderer.AddEntity(&m_plain);

		if (!m_quad.Init(device))
			return false;
		m_renderer.AddEntity(&m_quad.getEntity());

		m_walk.Init(&m_quad);
		m_camera.position = { 2.0f, 4.0f, -4.0f };
		m_camera.rotation = { 0.67f, -0.5f, 0.0f };
		return true;
	}
	void Scene::Frame(float deltaTime)
	{
		m_camera.Update(deltaTime);
		m_walk.Update(deltaTime);
		m_renderer.Render(m_window.getGfx(), m_camera);

		mth::float3 pos = m_quad.getEntity().position;
		m_window.setLabelText(std::to_wstring(pos.x).c_str(), 0);
		m_window.setLabelText(std::to_wstring(pos.y).c_str(), 1);
		m_window.setLabelText(std::to_wstring(pos.z).c_str(), 2);
	}
	void Scene::MessageHandler(MSG& msg)
	{
		m_camera.MessageHandler(msg);
		if (msg.message == WM_KEYDOWN && msg.wParam == VK_ESCAPE)
			PostQuitMessage(0);
	}
}