#include "texture.h"
#include <fstream>
#include <wincodec.h>

#pragma comment (lib, "windowscodecs.lib")

namespace gfx
{
	inline void ThrowIfFailed(HRESULT hr, const char* msg = nullptr)
	{
		if (FAILED(hr))
		{
			char buffer[32];
			sprintf_s(buffer, "0x%8x", hr);
			throw std::exception(msg ? (std::string(buffer) + msg).c_str() : buffer);
		}
	}

	void Texture::LoadTexture(Graphics& graphics, LPCWSTR filename)
	{		
		m_shaderResourceView.Release();

		AutoReleasePtr<IWICImagingFactory> factory;
		AutoReleasePtr<IWICBitmapDecoder> decoder;
		AutoReleasePtr<IWICBitmapFrameDecode> frame;
		AutoReleasePtr<IWICFormatConverter> converter;
		std::vector<BYTE> pixels;
		UINT imageWidth, imageHeight;

		ThrowIfFailed(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_IWICImagingFactory, reinterpret_cast<LPVOID*>(&factory)));
		ThrowIfFailed(factory->CreateDecoderFromFilename(filename, nullptr, GENERIC_READ, WICDecodeMetadataCacheOnLoad, &decoder));
		ThrowIfFailed(decoder->GetFrame(0, &frame));
		ThrowIfFailed(factory->CreateFormatConverter(&converter));
		ThrowIfFailed(converter->Initialize(frame, GUID_WICPixelFormat32bppPRGBA, WICBitmapDitherTypeNone, NULL, 0.0, WICBitmapPaletteTypeCustom));
		ThrowIfFailed(converter->GetSize(&imageWidth, &imageHeight));
		pixels.resize(4ull * imageWidth * imageHeight);
		ThrowIfFailed(converter->CopyPixels(nullptr, imageWidth * 4, imageWidth * imageHeight * 4, pixels.data()));

		AutoReleasePtr<ID3D11Texture2D> texture;
		D3D11_TEXTURE2D_DESC t2dd{};
		t2dd.Width = imageWidth;
		t2dd.Height = imageHeight;
		t2dd.MipLevels = 0;
		t2dd.ArraySize = 1;
		t2dd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		t2dd.SampleDesc.Count = 1;
		t2dd.SampleDesc.Quality = 0;
		t2dd.CPUAccessFlags = 0;
		t2dd.Usage = D3D11_USAGE_DEFAULT;
		t2dd.BindFlags = D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET;
		t2dd.MiscFlags = D3D11_RESOURCE_MISC_GENERATE_MIPS;
		ThrowIfFailed(graphics.getDevice()->CreateTexture2D(&t2dd, nullptr, &texture));
		graphics.getContext()->UpdateSubresource(texture, 0, nullptr, pixels.data(), static_cast<UINT>(imageWidth * 4ull), 0);

		D3D11_SHADER_RESOURCE_VIEW_DESC srvd{};
		srvd.Format = t2dd.Format;
		srvd.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		srvd.Texture2D.MostDetailedMip = 0;
		srvd.Texture2D.MipLevels = -1;
		ThrowIfFailed(graphics.getDevice()->CreateShaderResourceView(texture, &srvd, &m_shaderResourceView));
		graphics.getContext()->GenerateMips(m_shaderResourceView);
	}
	void Texture::CreateTexture(Graphics& graphics, void* data, int width, int height)
	{
		ID3D11Device* device = graphics.getDevice();
		ID3D11DeviceContext* context = graphics.getContext();
		HRESULT hr;
		AutoReleasePtr<ID3D11Texture2D> texture;
		D3D11_TEXTURE2D_DESC t2dd{};
		D3D11_SHADER_RESOURCE_VIEW_DESC srvd{};
		D3D11_SUBRESOURCE_DATA srd{};

		t2dd.Width = width;
		t2dd.Height = height;
		t2dd.MipLevels = 1;
		t2dd.ArraySize = 1;
		t2dd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		t2dd.SampleDesc.Count = 1;
		t2dd.SampleDesc.Quality = 0;
		t2dd.Usage = D3D11_USAGE_IMMUTABLE;
		t2dd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		t2dd.CPUAccessFlags = 0;
		t2dd.MiscFlags = 0;

		srd.pSysMem = data;
		srd.SysMemPitch = width * 4;
		srd.SysMemSlicePitch = 0;
		hr = device->CreateTexture2D(&t2dd, &srd, &texture);
		if (FAILED(hr))
		{
			auto error = GetLastError();
			throw std::exception("Failed to create texture");
		}

		srvd.Format = t2dd.Format;
		srvd.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
		srvd.Texture2D.MostDetailedMip = 0;
		srvd.Texture2D.MipLevels = -1;

		hr = device->CreateShaderResourceView(texture, &srvd, &m_shaderResourceView);
		if (FAILED(hr))
			throw std::exception("Failed to create shader resource view");
	}

	Texture::Texture(Graphics& graphics, LPCWSTR filename)
	{
		LoadTexture(graphics, filename);
	}
	Texture::Texture(Graphics& graphics, void* data, int width, int height)
	{
		CreateTexture(graphics, data, width, height);
	}
	Texture::P Texture::CreateTestTexture(Graphics& graphics)
	{
		unsigned r = 0xff0000ff;
		unsigned g = 0xff00ff00;
		unsigned b = 0xffff0000;
		unsigned k = 0xff000000;
		unsigned pixels[16][16];
		memset(pixels, 0xff, sizeof(pixels));
		for (int i = 0; i < 16; i++)
		{
			pixels[0][i] = k;
			pixels[0][i] = k;
			pixels[0][i] = k;
			pixels[15][i] = k;
			pixels[15][i] = k;
			pixels[15][i] = k;
			pixels[i][0] = k;
			pixels[i][0] = k;
			pixels[i][0] = k;
			pixels[i][15] = k;
			pixels[i][15] = k;
			pixels[i][15] = k;
			pixels[1][i] = k;
			pixels[1][i] = k;
			pixels[1][i] = k;
			pixels[14][i] = k;
			pixels[14][i] = k;
			pixels[14][i] = k;
			pixels[i][1] = k;
			pixels[i][1] = k;
			pixels[i][1] = k;
			pixels[i][14] = k;
			pixels[i][14] = k;
			pixels[i][14] = k;
		}
		{
			pixels[2][2] = r;
			pixels[3][2] = r;
			pixels[4][2] = r;
			pixels[5][2] = r;
			pixels[6][2] = r;
			pixels[2][3] = r;
			pixels[2][4] = r;
			pixels[4][3] = r;
			pixels[4][4] = r;
			pixels[3][5] = r;
			pixels[5][5] = r;
			pixels[6][5] = r;
		}
		{
			pixels[6][7] = g;
			pixels[6][8] = g;
			pixels[7][6] = g;
			pixels[8][6] = g;
			pixels[9][6] = g;
			pixels[10][7] = g;
			pixels[10][8] = g;
			pixels[10][9] = g;
			pixels[9][9] = g;
		}
		{
			pixels[9][10] = b;
			pixels[9][11] = b;
			pixels[9][12] = b;
			pixels[10][10] = b;
			pixels[10][13] = b;
			pixels[11][10] = b;
			pixels[11][11] = b;
			pixels[11][12] = b;
			pixels[12][10] = b;
			pixels[12][13] = b;
			pixels[13][10] = b;
			pixels[13][11] = b;
			pixels[13][12] = b;
		}
		return std::make_shared<Texture>(graphics, pixels, 16, 16);
	}
	void Texture::SetTextureToRender(Graphics& graphics, UINT index)
	{
		graphics.getContext()->PSSetShaderResources(index, 1, &m_shaderResourceView);
	}
}