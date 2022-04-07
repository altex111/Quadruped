#include "renderer.h"
#include <fstream>
#include <vector>

namespace gfx
{
	bool Model::Init(ID3D11Device *device, void *vertices, size_t vertexSize, void *indices, size_t indexSize)
	{
		m_indexCount = (UINT)(indexSize / sizeof(UINT));
		D3D11_BUFFER_DESC bufferDesc{};
		D3D11_SUBRESOURCE_DATA initData{};
		bufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		bufferDesc.ByteWidth = (UINT)vertexSize;
		bufferDesc.Usage = D3D11_USAGE_IMMUTABLE;
		initData.pSysMem = vertices;
		if (FAILED(device->CreateBuffer(&bufferDesc, &initData, &m_vertexBuffer)))
			return false;
		bufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
		bufferDesc.ByteWidth = (UINT)indexSize;
		bufferDesc.Usage = D3D11_USAGE_IMMUTABLE;
		initData.pSysMem = indices;
		if (FAILED(device->CreateBuffer(&bufferDesc, &initData, &m_indexBuffer)))
			return false;
		return true;
	}

	bool Model::Load(ID3D11Device *device, LPCWSTR filename)
	{
		std::ifstream infile;
		UINT vertexCount, indexCount;
		UINT shaderInputLayout, vertexSizeInBytes;
		std::vector<float> vertices;
		std::vector<UINT> indices;

		infile.open(filename, std::ios::in | std::ios::binary);
		if (!infile.good())
			return false;
		infile.read((char*)&shaderInputLayout, sizeof(UINT));
		infile.read((char*)&vertexCount, sizeof(UINT));
		infile.read((char*)&indexCount, sizeof(UINT));
		vertexSizeInBytes = 24;
		vertices.resize(vertexCount*vertexSizeInBytes / sizeof(float));
		indices.resize(indexCount);
		infile.read((char*)vertices.data(), vertices.size() * sizeof(float));
		infile.read((char*)indices.data(), indices.size() * sizeof(UINT));
		infile.close();

		return Init(device, vertices.data(), vertices.size() * sizeof(float), indices.data(), indexCount * sizeof(UINT));
	}
	void Model::Shutdown()
	{
		m_indexCount = 0;
		m_vertexBuffer.Release();
		m_indexBuffer.Release();
	}
	void Model::SetBuffers(ID3D11DeviceContext *context)
	{
		UINT stride = sizeof(VerteyType);
		UINT offset = 0;

		context->IASetVertexBuffers(0, 1, &m_vertexBuffer, &stride, &offset);
		context->IASetIndexBuffer(m_indexBuffer, DXGI_FORMAT_R32_UINT, 0);
	}
	void Model::Draw(ID3D11DeviceContext *context)
	{
		context->DrawIndexed(m_indexCount, 0, 0);
	}
}