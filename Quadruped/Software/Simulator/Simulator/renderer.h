#pragma once

#include "entity.h"
#include "camera.h"

namespace gfx
{
	class Renderer
	{
	protected:
		AutoReleasePtr<ID3D11VertexShader> m_vertexShader;
		AutoReleasePtr<ID3D11InputLayout> m_inputLayout;
		AutoReleasePtr<ID3D11PixelShader> m_pixelShader;
		AutoReleasePtr<ID3D11Buffer> m_vsMatrixBuffer;
		AutoReleasePtr<ID3D11Buffer> m_psLightBuffer;
		AutoReleasePtr<ID3D11Buffer> m_psColorBuffer;

		std::vector<Entity*> m_entities;

	private:
		bool CreateVertexShader(ID3D11Device *device);
		bool CreatePixelShader(ID3D11Device *device);
		bool CreateConstantBuffers(ID3D11Device *device);

		bool WriteMatrixBuffer(ID3D11DeviceContext *context, void *dataptr);
		bool WriteLightBuffer(ID3D11DeviceContext *context, void *dataptr);
		bool WriteColorBuffer(ID3D11DeviceContext *context, void *dataptr);

		void RenderEntity(ID3D11DeviceContext *context, Entity *entity, mth::matrix matrixBuffer[], mth::matrix baseMatrix);

	public:
		Renderer() = default;
		bool Init(Graphics& graphics);
		void AddEntity(Entity *entity);
		void RemoveEntity(Entity *entity);
		void ClearEntities();

		void Render(Graphics& graphics, Camera& camera);
	};
}