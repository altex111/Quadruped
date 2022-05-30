#include "entity.h"

namespace gfx
{
	bool Entity::Init(ID3D11Device *device, LPCWSTR modelPath, mth::float4 color)
	{
		m_color = color;
		return m_model.Load(device, modelPath);
	}

	void Entity::AddSubpart(Entity *entity)
	{
		m_subparts.push_back(entity);
	}

	void Entity::Render(ID3D11DeviceContext *context)
	{
		m_model.SetBuffers(context);
		m_model.Draw(context);
	}
}
