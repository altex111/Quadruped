#pragma once

#include "cmain.h"

namespace util {

template <typename T, uint32_t S>
class fifo {
	T m_buffer[S];
	int32_t m_begin;
	int32_t m_end;
	int32_t m_size;

public:
	fifo()
	{
		clear();
	}

	int32_t size()
	{
		return m_size;
	}

	bool push(T& t)
	{
		if (m_size < (int32_t)S)
		{
			m_buffer[m_end] = t;
			m_end = (m_end + 1) % S;
			m_size++;
			return true;
		}
		return false;
	}
	bool pop(T& t)
	{
		if (m_size == 0)
			return false;
		t = m_buffer[m_begin];
		m_begin = (m_begin + 1) % S;
		m_size--;
		return true;
	}

	void clear()
	{
		m_begin = 0;
		m_end = 0;
		m_size = 0;
	}
	bool empty()
	{
		return m_size == 0;
	}
};

}
