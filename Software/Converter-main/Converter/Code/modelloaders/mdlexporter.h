#pragma once

#include "mdlloader.h"

namespace gfx
{
	class MDLExporter :public ModelLoader
	{
	public:
		void ExportMDL(LPCWSTR filename);
	};
}