#include "mdlloader.h"

namespace gfx
{
	void MDLLoader::LoadMDL(LPCWSTR filename, UINT modelType)
	{
		UINT vertexCount, indexCount;
		UINT shaderInputLayout, vertexSizeInBytes;
		std::vector<float> vertices;
		std::vector<UINT> indices;

		std::ifstream infile(filename, std::ios::in | std::ios::binary);
		if (!infile.good())
			throw std::exception(std::string("Failed to load file: " + ToStr(filename)).c_str());

		infile.read((char*)&shaderInputLayout, sizeof(UINT));
		infile.read((char*)&vertexCount, sizeof(UINT));
		infile.read((char*)&indexCount, sizeof(UINT));
		vertexSizeInBytes = 24;
		vertices.resize(vertexCount * vertexSizeInBytes / sizeof(float));
		indices.resize(indexCount);
		infile.read((char*)vertices.data(), vertices.size() * sizeof(float));
		infile.read((char*)indices.data(), indices.size() * sizeof(UINT));
		infile.close();

		std::vector<Vertex_PTMB> vertexPTMB(vertexCount);

		for (UINT i = 0; i < vertexCount; i++)
		{
			vertexPTMB[i].position.x = vertices[6 * i + 0];
			vertexPTMB[i].position.y = vertices[6 * i + 1];
			vertexPTMB[i].position.z = vertices[6 * i + 2];
			vertexPTMB[i].normal.x = vertices[6 * i + 3];
			vertexPTMB[i].normal.y = vertices[6 * i + 4];
			vertexPTMB[i].normal.z = vertices[6 * i + 5];
		}
		Create(vertexPTMB.data(), vertexCount, indices.data(), indexCount, ModelType::PN);
	}
}