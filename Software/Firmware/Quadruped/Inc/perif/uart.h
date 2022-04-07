#pragma once

#include "util/fifo.h"
#include <functional>

namespace quad
{

class UART
{
	USART_TypeDef *m_uart;
	util::fifo<uint8_t, 64> m_inData;
	util::fifo<uint8_t, 64> m_outData;

private:
	void StartDataTransmit();

public:
	UART(USART_TypeDef *uart);
	template <typename T>
	void SendData(T data)
	{
		SendData((uint8_t*)&data, sizeof(T));
	}
	void SendData(uint8_t *data, uint32_t size);
	void SendData(uint8_t *str);
	bool ReadBuffer(uint8_t *data, uint32_t len);
	void InterruptHandler();
};

}
