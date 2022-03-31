#pragma once

#include "cmain.h"
#include <string.h>
#include "util/string_fifo.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

namespace util {

template <typename T, uint32_t S>
class fifo {
protected:
	StreamBufferHandle_t m_buffer;

#if DEBUG_CODE_FIFO
	bool push_overflow = false;
#endif

public:
	fifo()
	{
		m_buffer = xStreamBufferCreate((S*sizeof(T)), 0u);
		clear();
	}

	int32_t size()
	{
		return (xStreamBufferBytesAvailable(m_buffer) / sizeof(T));
	}

	bool push(T& t)
	{
		if(xStreamBufferSend(m_buffer, &t, sizeof(T), 0u))
		{
			return true;
		}
#if DEBUG_CODE_FIFO
	push_overflow = true;
#endif
		return false;
	}

	bool push_ISR(T& t)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if(xStreamBufferSendFromISR(m_buffer, &t, sizeof(T), &xHigherPriorityTaskWoken))
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			return true;
		}
#if DEBUG_CODE_FIFO
	push_overflow = true;
#endif
		return false;
	}

	bool pop(T& t)
	{
		if(xStreamBufferReceive(m_buffer, &t, sizeof(T), 0u))
		{
			return true;
		}
		return false;
	}

	bool pop_ISR(T& t)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if(xStreamBufferReceiveFromISR(m_buffer, &t, sizeof(T), &xHigherPriorityTaskWoken))
		{
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			return true;
		}
		return false;
	}

	template<uint32_t D>
	uint32_t copy2fifo(fifo<T, D>& fifo)
	{
		uint32_t count;
		T tmp;
		while(pop(tmp))
		{
			fifo.push(tmp);
			count++;
		}
		return count;
	}

	template<uint32_t D>
	uint32_t copy2fifo(string_fifo<D>& fifo)
	{
		uint32_t count;
		T tmp;
		while(pop(tmp))
		{
			fifo.push(tmp);
			count++;
		}
		return count;
	}

	template<typename H, uint32_t D>
	uint32_t copy2fifo(fifo<H, D>& fifo)
	{
		if(sizeof(H) < sizeof(T))
		{
			return 0u;
		}

		uint8_t divisor = sizeof(H) / sizeof(T);
		uint32_t count = 0u;
		T tmp = 0u;
		H tmp2 = 0u;

		while(this->size() >= divisor)
		{
			for(uint8_t iter=0u; iter < divisor; iter++)
			{
				pop(tmp);
				tmp2 |= (tmp << (iter*8*sizeof(T)));
			}
			fifo.push(tmp2);
			tmp2 = 0u;
			count++;
		}
		return count;
	}

	void clear()
	{
		xStreamBufferReset(m_buffer);
	}
	bool empty()
	{
		return (pdTRUE == xStreamBufferIsEmpty(m_buffer));
	}

#if DEBUG_CODE_ACTIVE
	int32_t begin()
	{
		return m_begin;
	}
	T* buffer()
	{
		return m_buffer;
	}
#endif

};

}
