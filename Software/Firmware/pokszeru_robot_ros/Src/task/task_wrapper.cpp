/*
 * Task.cpp
 *
 *  Created on: 2017. okt. 10.
 *      Author: Nagy Ákos
 */

#include "task/task_wrapper.h"

namespace task
{

Task::Task(const char* name, uint16_t stackSize, uint32_t priority)
{
	this->name = name;
	this->stackSize = stackSize;
	this->priority = priority;

	taskID = NULL;
}

Task::~Task()
{
	Cleanup();
}

bool Task::Create()
{
	if (xTaskCreate(TaskAdapter, name, stackSize, this, priority, &taskID) != pdPASS)
		return false;

	if (taskID == NULL)
		return false;

	return true;
}

bool Task::Cleanup()
{
	if (taskID != NULL)
		vTaskDelete(taskID);

	taskID = NULL;
	return true;
}

void Task::StartScheduler()
{
	vTaskStartScheduler();
}

void Task::TaskAdapter(void* task)
{
    Task* t = static_cast<Task*>(task);

    t->Process();
    t->Cleanup();

    configASSERT(!"Returned from task's Process!");
}

}
