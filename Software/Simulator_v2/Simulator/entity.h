#pragma once

#include "model.h"
#include "position.h"
#include <vector>

namespace gfx
{
	class Entity :public mth::Position
	{
	protected:
		Model m_model;
		mth::float4 m_color;
		std::vector<Entity*> m_subparts;

	public:
		bool Init(ID3D11Device *device, LPCWSTR modelPath, mth::float4 color);
		void AddSubpart(Entity *entity);
		inline void setColor(mth::float4 color) { m_color = color; }
		inline mth::float4 getColor() { return m_color; }

		inline std::vector<Entity*>& getSubparts() { return m_subparts; }
		void Render(ID3D11DeviceContext *context);
	};
}