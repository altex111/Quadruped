#include "mdlexporter.h"

namespace gfx
{
	void MDLExporter::ExportMDL(LPCWSTR filename)
	{
		UINT shaderInputLayout = 9;
		UINT vertexCount = m_vertices.size() * sizeof(float) / m_vertexSizeInBytes;
		UINT indexCount = m_indices.size();

		std::ofstream outfile(filename, std::ios::out | std::ios::binary);
		if (!outfile.good())
			throw std::exception(std::string("Failed to open file: " + ToStr(filename)).c_str());

		UINT positionOffset = ModelType::PositionOffset(m_modelType);
		UINT normalOffset = ModelType::NormalOffset(m_modelType);
		std::vector<float> vertices(vertexCount * 6);
		for (UINT i = 0; i < vertexCount; i++)
		{
			vertices[6 * i + 0] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + positionOffset + 0].f;
			vertices[6 * i + 1] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + positionOffset + 1].f;
			vertices[6 * i + 2] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + positionOffset + 2].f;
			vertices[6 * i + 3] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + normalOffset + 0].f;
			vertices[6 * i + 4] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + normalOffset + 1].f;
			vertices[6 * i + 5] = m_vertices[m_vertexSizeInBytes / sizeof(float) * i + normalOffset + 2].f;
		}

		outfile.write((char*)&shaderInputLayout, sizeof(UINT));
		outfile.write((char*)&vertexCount, sizeof(UINT));
		outfile.write((char*)&indexCount, sizeof(UINT));
		outfile.write((char*)vertices.data(), vertices.size() * sizeof(float));
		outfile.write((char*)m_indices.data(), m_indices.size() * sizeof(UINT));
		outfile.close();
	}
}