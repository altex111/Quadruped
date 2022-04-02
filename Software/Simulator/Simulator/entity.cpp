#include "entity.h"

namespace gfx
{
	bool Entity::Init(ID3D11Device *device, LPCWSTR modelPath, mth::float4 color, 
		mth::float3 rotOffset, mth::float3 posOffset)
	{
		m_color = color;
		this->posOffset = posOffset;
		this->rotOffset = rotOffset;
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
