#include "perif/io.h"
#include "cmain.h"

namespace quad
{
IO::Button::Button(GPIO_TypeDef *port, uint32_t pin)
:	m_port(port),
	m_pin(pin),
	m_callback(),
	m_lastState(true){}
bool IO::Button::isDown()
{
	return (bool)(LL_GPIO_ReadInputPort(m_port) & m_pin);
}
void IO::Button::setCallback(std::function<void(void)> callback)
{
	m_callback = callback;
}
void IO::Button::Update()
{
	bool btnState = isDown();
	if ((bool)m_callback && !m_lastState && btnState)
		m_callback();
	m_lastState = btnState;
}

IO::LED::LED(GPIO_TypeDef *port, uint32_t pin)
:	m_port(port),
	m_pin(pin){}
void IO::LED::Set()
{
	LL_GPIO_SetOutputPin(m_port, m_pin);
}
void IO::LED::Reset()
{
	LL_GPIO_ResetOutputPin(m_port, m_pin);
}
void IO::LED::Toggle()
{
	LL_GPIO_TogglePin(m_port, m_pin);
}
void IO::LED::SetTo(bool on)
{
	if (on) Set();
	else Reset();
}

IO::Button IO::btn1(GPIOB, LL_GPIO_PIN_9);
IO::Button IO::btn2(GPIOB, LL_GPIO_PIN_8);
IO::LED IO::led1(GPIOB, LL_GPIO_PIN_10);
IO::LED IO::led2(GPIOB, LL_GPIO_PIN_11);
IO::LED IO::led3(GPIOC, LL_GPIO_PIN_3);
IO::LED IO::ledAccu(GPIOB, LL_GPIO_PIN_12);
void IO::Update()
{
	btn1.Update();
	btn2.Update();
}
}
