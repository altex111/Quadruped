/*
 * task_adc.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Mass�r L�r�nt
 */

#pragma once

#include "task/task_wrapper.h"

namespace task
{

class TaskADC : public Task
{
private:

public:
	TaskADC(const char* name, uint16_t stackSize, uint32_t priority);

    void Process();
};

}
