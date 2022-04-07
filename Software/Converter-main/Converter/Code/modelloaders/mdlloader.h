#pragma once

#include "modelloader.h"

namespace gfx
{
	class MDLLoader :public ModelLoader
	{
	public:
		void LoadMDL(LPCWSTR filename, UINT modelType);
	};
}