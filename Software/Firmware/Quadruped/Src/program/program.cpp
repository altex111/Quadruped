#include "program/program.h"

namespace quad
{

Program::Program()
:	m_PCcomm(UART4),
	m_quad(),
	m_walkScipt(m_quad),
	m_quadEnabled(false){}

}
