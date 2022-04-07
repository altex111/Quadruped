#pragma once

#include "cmain.h"
#include <string.h>

namespace util {

template <uint32_t S>
class string_fifo{

protected:
	char m_buffer[S];
	int32_t m_begin;
	int32_t m_end;
	int32_t m_size;

#if DEBUG_CODE_FIFO
	bool push_overflow = false;
#endif

public:
	string_fifo()
	{
		clear();
	}

	int32_t size()
	{
		return m_size;
	}

	bool push(char& t)
	{
		if (m_size < (int32_t)S)
		{
			m_buffer[m_end] = t;
			m_end = (m_end + 1) % S;
			m_size++;
			return true;
		}
#if DEBUG_CODE_FIFO
	push_overflow = true;
#endif
		return false;
	}
	bool pop(char& t)
	{
		if (m_size == 0)
			return false;
		t = m_buffer[m_begin];
		m_begin = (m_begin + 1) % S;
		m_size--;
		return true;
	}

	template<uint32_t D>
	uint32_t copy2fifo(string_fifo<D>& fifo)
	{
		uint32_t count;
		char tmp;
		while(pop(tmp))
		{
			fifo.push(tmp);
			count++;
		}
		return count;
	}

	void clear()
	{
		m_begin = 0;
		m_end = 0;
		m_size = 0;
	}
	bool empty()
	{
		return (0 == m_size);
	}

	void insertTermination()
	{
		this->m_buffer[this->m_end] = '\0';
	}

	void reorder()
	{
		if(0 != this->m_size && 0 != this->m_begin)
		{
			char reorder_tmp[S];
			char tmp_null = '\0';
			int32_t reorder_size = this->m_size;

			for(int32_t looper = 0; this->pop(reorder_tmp[looper]); looper++)
			{

			}

			this->clear();

			for(int32_t looper = 0; looper < reorder_size; looper++)
			{
				this->push(reorder_tmp[looper]);
			}

			/* Add null character for the compare to avoid duplicated strings, this terminating character will prevent the strstr function to falsely detect reordered parts of the input */
			this->m_buffer[this->m_end] = tmp_null;
		}
	}

	int16_t compare(char* str)
	{
		char* tmp;

		/* Call reoder() function to avoid not continuous data in circular buffer */
		reorder();
		tmp = strstr(this->m_buffer, str);

		if(NULL != tmp)
		{
			return (int16_t)(tmp - this->m_buffer);
		}
		else
		{
			return -1;
		}
	}

	bool esp_size_arrived()
	{
		/* Call reoder() function to avoid not continuous data in circular buffer */
		reorder();

		for(uint32_t looper = this->m_begin; looper < this->m_end ;looper++)
		{
			if(':' == this->m_buffer[looper])
			{
				return true;
			}
		}
		return false;
	}
};
}
