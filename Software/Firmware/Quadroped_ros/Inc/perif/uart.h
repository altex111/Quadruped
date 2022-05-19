#pragma once

#include "util/fifo.h"
#include <functional>
#include "program/common.h"

namespace quadroped
{

class UART
{
	USART_TypeDef *m_uart;
	util::fifo<char, UART_INDATA_LENGTH> m_inData;
	util::fifo<char, UART_OUTDATA_LENGTH> m_outData;

private:
	void StartDataTransmit();
	static IRQn_Type GetUARTInterrupt(USART_TypeDef* uart)
	{
		IRQn_Type returnValue;

		if(USART3 == uart)
		{
			returnValue = USART3_IRQn;
		}
		else if(USART1 == uart)
		{
			returnValue = USART1_IRQn;
		}
		else if(UART4 == uart)
		{
			returnValue = UART4_IRQn;
		}
		else
		{
			returnValue = PendSV_IRQn; //todo
		}

		return returnValue;
	}

public:
	UART(USART_TypeDef *uart);
	template <typename T>
	void SendData(T data)
	{
		SendData((char*)&data, sizeof(T));
	}
	void SendData(char *data, uint32_t size);
	void SendString(char *str)
	{
		SendData(str);
	}
	void SendData(char *str);
	bool ReadBuffer(char *data, uint32_t len);
	void InterruptHandler();
	void ClearInFifo();
	int32_t GetInSize();
	int32_t GetOutSize();

	template<uint32_t S>
	uint32_t CopyInFifo(util::fifo<char, S>& fifo)
	{
		return m_inData.copy2fifo(fifo);
	}

	template<uint32_t S>
	uint32_t CopyInFifo(util::string_fifo<S>& fifo)
	{
		return  m_inData.copy2fifo(fifo);
	}

	template<uint32_t S>
	uint32_t CopyInFifo(util::fifo<uint16_t, S>& fifo)
	{
		return m_inData.copy2fifo(fifo);
	}

	void ClearISRErrors();
};

}
