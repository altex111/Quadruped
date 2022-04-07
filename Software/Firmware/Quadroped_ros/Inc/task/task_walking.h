/*
 * task_walking.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#pragma once

#include "task/task_wrapper.h"
#include "program/walk.h"
#include "message_buffer.h"

namespace task
{

class TaskWalk : public Task
{
private:
    quad::Walk walk;
    MessageBufferHandle_t RosToWalkMessage;
    WalkCommand walk_Command;

public:
	TaskWalk(const char* name, uint16_t stackSize, uint32_t priority, MessageBufferHandle_t& RosToWalkMessage);

    void Process();
};

}
