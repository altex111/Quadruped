#include "perif/uart.h"

namespace quadroped
{
UART::UART(USART_TypeDef *uart)
:m_uart(uart), m_inData(), m_outData()
{}
void UART::StartDataTransmit()
{
	LL_USART_EnableIT_TXE(m_uart);
}
void UART::SendData(char *data, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++)
		m_outData.push(data[i]);
	StartDataTransmit();
}
void UART::SendData(char *str)
{
	uint32_t len = 0;
	for (len = 0; str[len]; len++);
	SendData(str, len);
}
bool UART::ReadBuffer(char *data, uint32_t len)
{
	if (m_inData.size() < (int32_t)len)
		return false;
	for (uint32_t i = 0; i < len; i++)
		m_inData.pop(data[i]);
	return true;
}
void UART::InterruptHandler()
{
	char ch;

	if (LL_USART_IsActiveFlag_RXNE(m_uart) &&  LL_USART_IsEnabledIT_RXNE(m_uart))
	{
		ch = LL_USART_ReceiveData8(m_uart);
		m_inData.push_ISR(ch);
	}

	if (LL_USART_IsActiveFlag_TXE(m_uart) &&  LL_USART_IsEnabledIT_TXE(m_uart))
	{
		if (m_outData.pop_ISR(ch))
			LL_USART_TransmitData8(m_uart, ch);
		else
			LL_USART_DisableIT_TXE(m_uart);
	}

	ClearISRErrors();
}
void UART::ClearInFifo()
{
	m_inData.clear();
}
int32_t UART::GetInSize()
{
	return m_inData.size();
}
int32_t UART::GetOutSize()
{
	return m_outData.size();
}

void UART::ClearISRErrors()
{
	uint32_t isrflags = READ_REG(m_uart->ISR);
	uint32_t cr1its = READ_REG(m_uart->CR1);
	uint32_t cr3its = READ_REG(m_uart->CR3);

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
		m_uart->ICR = USART_ICR_FECF;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
    	m_uart->ICR = USART_ICR_NECF;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(((isrflags & USART_ISR_ORE) != RESET) && (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
	{
    	m_uart->ICR = USART_ICR_ORECF;
	}
}
}
