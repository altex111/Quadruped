/*
 * task_lidar.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Mass�r L�r�nt
 */

#pragma once

#include "task/task_wrapper.h"
#include "perif/lidar.h"
#include "program/common.h"
/* todo delete*/
#include "event_groups.h"
#include "message_buffer.h"
/* todo delete*/

namespace task
{

class TaskLIDAR : public Task
{
private:
	quadroped::LIDAR lidar;
	EventGroupHandle_t SchedulerEventGroup;
	MessageBufferHandle_t LidarToRosMessage;

public:
	TaskLIDAR(const char* name, uint16_t stackSize, uint32_t priority, EventGroupHandle_t& SchedulerEventGroup, MessageBufferHandle_t& LidarToRosMessage);

    void Process();

    inline quadroped::LIDAR& getLIDAR() {return lidar;}
};

}
