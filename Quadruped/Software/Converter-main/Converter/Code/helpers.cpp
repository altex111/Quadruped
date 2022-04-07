#include "helpers.h"
#include <filesystem>

std::wstring g_ExeFolder;

std::wstring ToWStr(const char* str)
{
	std::wstring r;
	for (int i = 0; str[i]; i++)
		r += static_cast<wchar_t>(str[i]);
	return r;
}

std::string ToStr(const wchar_t* str)
{
	std::string r;
	for (int i = 0; str[i]; i++)
		r += static_cast<char>(str[i]);
	return r;
}

void SetExeFolderName(HINSTANCE hInstance)
{
	WCHAR path[MAX_PATH];
	GetModuleFileName(hInstance, path, MAX_PATH);

	int lastSlashIndex = -1;
	for (int i = 0; path[i]; i++)
		if (path[i] == '\\' || path[i] == '/')
			lastSlashIndex = i;
	for (int i = 0; i <= lastSlashIndex; i++)
		g_ExeFolder += (WCHAR)path[i];
}

std::wstring ResolveFilename(std::wstring filename)
{
	if (std::filesystem::exists(g_ExeFolder + filename))
		return g_ExeFolder + filename;
	return filename;
}

std::wstring GetFileExtension(LPCWSTR filename)
{
	std::wstring extension;
	int lastDotIndex = -1;
	for (int i = 0; filename[i]; i++)
		if (filename[i] == '.')
			lastDotIndex = i;

	if (lastDotIndex != -1)
		for (int i = lastDotIndex + 1; filename[i] != '\0' && filename[i] != '\"'; i++)
			extension += filename[i];
	return extension;
}
