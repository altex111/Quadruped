/*
 * Task.h
 *
 *  Created on: 2017. okt. 10.
 *      Author: Nagy Ákos
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"

namespace task
{

class Task
{
public:
					Task(const char* name, uint16_t stackSize, uint32_t priority);
	virtual 		~Task();
	//Task
	virtual bool	Create();
	virtual bool	Cleanup();
	virtual void 	Process() = 0;
	//Scheduler
	static void		StartScheduler();
	static void 	TaskAdapter(void* task);
protected:
	const char*		name;
	uint32_t		priority;
	uint32_t		stackSize;
	TaskHandle_t	taskID;
};

}
