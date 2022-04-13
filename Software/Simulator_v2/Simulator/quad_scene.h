#pragma once

#include "window.h"
#include "quad_quad.h"
#include "quad_movement.h"
#include "quad_walkscript.h"
#include "renderer.h"
#include "camera.h"

namespace quad
{
	class Scene
	{
		gfx::Window& m_window;
		gfx::Renderer m_renderer;

		gfx::Camera m_camera;
		gfx::Entity m_plain;

		Quadruped m_quad;
		WalkManager m_walk;
		bool m_running;

	public:
		Scene(gfx::Window& window);

		bool Start();
		void Frame(float deltaTime);
		void MessageHandler(MSG& msg);
	};
}