/*
 * task_ros.h
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#pragma once

#include "task/task_wrapper.h"
#include <ros/ros_wrapper.h>
/* todo delete*/
#include "message_buffer.h"
/* todo delete*/

namespace task
{

class TaskROS : public Task
{
private:
    ros::ROSNode node;
    EventGroupHandle_t SchedulerEventGroup;
    MessageBufferHandle_t ImuToRosMessageQuaternion;
    MessageBufferHandle_t ImuToRosMessageDebug;
    MessageBufferHandle_t RosToImuMessageDebug;
    MessageBufferHandle_t LidarToRosMessage;
    MessageBufferHandle_t RosToWalkMessage;

public:
    TaskROS(const char* name, uint16_t stackSize, uint32_t priority,
    		EventGroupHandle_t& SchedulerEventGroup,
			MessageBufferHandle_t& ImuToRosMessageQuaternion,
			MessageBufferHandle_t& ImuToRosMessageDebug,
			MessageBufferHandle_t& RosToImuMessageDebug,
			MessageBufferHandle_t& LidarToRosMessage,
			MessageBufferHandle_t& RosToWalkMessage);

    void Process();

    inline ros::ROSNode& getROSNode() {return node;}


};

}
