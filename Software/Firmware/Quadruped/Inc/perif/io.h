#pragma once

#include "cmain.h"
#include <functional>

namespace quad
{

struct IO
{
	class Button
	{
		GPIO_TypeDef *m_port;
		uint32_t m_pin;
		std::function<void(void)> m_callback;
		bool m_lastState;

	public:
		Button(GPIO_TypeDef *port, uint32_t pin);
		bool isDown();
		void setCallback(std::function<void(void)> callback);
		void Update();
	};

	class LED
	{
		GPIO_TypeDef *m_port;
		uint32_t m_pin;

	public:
		LED(GPIO_TypeDef *port, uint32_t pin);
		void Set();
		void Reset();
		void Toggle();
		void SetTo(bool on);
	};

	static Button btn1;
	static Button btn2;
	static LED led1;
	static LED led2;
	static LED led3;
	static LED ledAccu;

	static void Update();
};

}
