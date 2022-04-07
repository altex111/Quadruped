#pragma once

#include "quad/walkscript.h"
#include "perif/io.h"
#include "perif/uart.h"
#include "cmain.h"
#include <cmath>

namespace quad
{

class Program
{
protected:
	UART m_PCcomm;
	quad::Quadruped m_quad;
	quad::WalkScript m_walkScipt;
	bool m_quadEnabled;

public:
	Program();
	inline UART& getPCComm() {return m_PCcomm;}
	virtual ~Program() = default;
	virtual void Start() = 0;
	virtual void Loop() = 0;
};

}
