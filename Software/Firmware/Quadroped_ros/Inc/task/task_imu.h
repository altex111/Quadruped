/*
 * task_imu.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#pragma once

#include "task/task_wrapper.h"
#include "perif/imu.h"
/* todo delete*/
#include "message_buffer.h"
#include "event_groups.h"
/* todo delete*/

namespace task
{

class TaskIMU : public Task
{
private:
    quad::IMU imu;
    EventGroupHandle_t SchedulerEventGroup;
    MessageBufferHandle_t ImuToRosMessageQuaternion;
    MessageBufferHandle_t ImuToRosMessageDebug;
    MessageBufferHandle_t RosToImuMessageDebug;

public:
	TaskIMU(const char* name, uint16_t stackSize, uint32_t priority,
			EventGroupHandle_t& SchedulerEventGroup,
			MessageBufferHandle_t& ImuToRosMessageQuaternion,
			MessageBufferHandle_t& ImuToRosMessageDebug,
			MessageBufferHandle_t& RosToImuMessageDebug);

    void Process();

    inline quad::IMU& getIMU() {return imu;}
};

}
