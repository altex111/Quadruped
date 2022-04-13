#include "renderer.h"
#include <fstream>

namespace gfx
{
	ID3DBlob* LoadShaderCode(LPCWSTR filename)
	{
		HRESULT result;
		std::fstream f(filename, std::ios::in | std::ios::binary);
		if (!f.good())
			return nullptr;
		auto begin = f.tellg();
		f.seekg(0, std::ios::end);
		auto end = f.tellg();
		f.seekg(0);
		ID3DBlob* shaderCode = nullptr;
		result = D3DCreateBlob((SIZE_T)(end - begin), &shaderCode);
		if (FAILED(result))
			return nullptr;
		f.read((char*)shaderCode->GetBufferPointer(), shaderCode->GetBufferSize());
		f.close();
		return shaderCode;
	}

	bool Renderer::CreateVertexShader(ID3D11Device *device)
	{
		AutoReleasePtr<ID3DBlob> shaderBuffer = LoadShaderCode(L"resources/vertexShader.cso");
		if (shaderBuffer == nullptr)
			return false;
		if (FAILED(device->CreateVertexShader(shaderBuffer->GetBufferPointer(), shaderBuffer->GetBufferSize(), NULL, &m_vertexShader)))
			return false;
		D3D11_INPUT_ELEMENT_DESC inputLayoutDesc[2];
		inputLayoutDesc[0] = { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 };
		inputLayoutDesc[1] = { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 };
		if (FAILED(device->CreateInputLayout(inputLayoutDesc, 2, shaderBuffer->GetBufferPointer(), shaderBuffer->GetBufferSize(), &m_inputLayout)))
			return false;
		return true;
	}
	bool Renderer::CreatePixelShader(ID3D11Device *device)
	{
		AutoReleasePtr<ID3DBlob> shaderBuffer = LoadShaderCode(L"resources/pixelShader.cso");
		if (shaderBuffer == nullptr)
			return false;
		if (FAILED(device->CreatePixelShader(shaderBuffer->GetBufferPointer(), shaderBuffer->GetBufferSize(), nullptr, &m_pixelShader)))
			return false;
		return true;
	}
	bool Renderer::CreateConstantBuffers(ID3D11Device *device)
	{
		D3D11_BUFFER_DESC bufferDesc;
		bufferDesc.Usage = D3D11_USAGE_DYNAMIC;
		bufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		bufferDesc.MiscFlags = 0;
		bufferDesc.StructureByteStride = 0;
		bufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

		bufferDesc.ByteWidth = sizeof(mth::matrix) * 2;
		if (FAILED(device->CreateBuffer(&bufferDesc, NULL, &m_vsMatrixBuffer)))
			return false;

		bufferDesc.ByteWidth = sizeof(float) * 8;
		if (FAILED(device->CreateBuffer(&bufferDesc, NULL, &m_psLightBuffer)))
			return false;

		bufferDesc.ByteWidth = sizeof(float) * 4;
		if (FAILED(device->CreateBuffer(&bufferDesc, NULL, &m_psColorBuffer)))
			return false;

		return true;
	}
	bool Renderer::WriteMatrixBuffer(ID3D11DeviceContext *context, void *dataptr)
	{
		D3D11_MAPPED_SUBRESOURCE resource;
		if (SUCCEEDED(context->Map(m_vsMatrixBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource)))
		{
			memcpy(resource.pData, dataptr, sizeof(mth::matrix) * 2);
			context->Unmap(m_vsMatrixBuffer, 0);
			return true;
		}
		return false;
	}
	bool Renderer::WriteLightBuffer(ID3D11DeviceContext *context, void *dataptr)
	{
		D3D11_MAPPED_SUBRESOURCE resource;
		if (SUCCEEDED(context->Map(m_psLightBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource)))
		{
			memcpy(resource.pData, dataptr, sizeof(float) * 8);
			context->Unmap(m_psLightBuffer, 0);
			return true;
		}
		return false;
	}
	bool Renderer::WriteColorBuffer(ID3D11DeviceContext *context, void *dataptr)
	{
		D3D11_MAPPED_SUBRESOURCE resource;
		if (SUCCEEDED(context->Map(m_psColorBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource)))
		{
			memcpy(resource.pData, dataptr, sizeof(float) * 4);
			context->Unmap(m_psColorBuffer, 0);
			return true;
		}
		return false;
	}
	bool Renderer::Init(Graphics& graphics)
	{
		auto device = graphics.getDevice();
		auto context = graphics.getContext();

		if (!CreateVertexShader(device) ||
			!CreatePixelShader(device) ||
			!CreateConstantBuffers(device))
			return false;

		context->VSSetShader(m_vertexShader, NULL, 0);
		context->IASetInputLayout(m_inputLayout);
		context->VSSetConstantBuffers(0, 1, &m_vsMatrixBuffer);
		context->PSSetShader(m_pixelShader, NULL, 0);
		context->PSSetConstantBuffers(0, 1, &m_psLightBuffer);
		context->PSSetConstantBuffers(1, 1, &m_psColorBuffer);

		return true;
	}
	void Renderer::AddEntity(Entity *entity)
	{
		m_entities.push_back(entity);
	}
	void Renderer::RemoveEntity(Entity *entity)
	{
		for (auto e = m_entities.begin(); e != m_entities.end(); e++)
		{
			if (*e == entity)
			{
				m_entities.erase(e);
				return;
			}
		}
	}
	void Renderer::ClearEntities()
	{
		m_entities.clear();
	}
	void Renderer::RenderEntity(ID3D11DeviceContext *context, Entity *entity, mth::matrix matrixBuffer[], mth::matrix baseMatrix)
	{
		baseMatrix = baseMatrix*entity->GetWorldMatrix();
		matrixBuffer[0] = baseMatrix;
		WriteMatrixBuffer(context, matrixBuffer);
		mth::float4 c = entity->getColor();
		WriteColorBuffer(context, &c);
		entity->Render(context);
		for (auto e : entity->getSubparts())
			RenderEntity(context, e, matrixBuffer, baseMatrix);
	}
	void Renderer::Render(Graphics& graphics, Camera& camera)
	{
		auto context = graphics.getContext();

		graphics.BeginFrame(0.5f, 0.75f, 0.96f);

		mth::matrix matrixBuffer[2];
		matrixBuffer[1] = camera.getCameraMatrix();

		mth::float3 campos = camera.position;
		float lightBuffer[8] = {
			1.0f, 1.0f, 1.0f, 1.0f,
			campos.x, campos.y, campos.z,
			0.5f
		};
		WriteLightBuffer(context, lightBuffer);

		for (auto& e : m_entities)
			RenderEntity(context, e, matrixBuffer, mth::float4x4::Identity());

		graphics.EndFrame();
	}
}
