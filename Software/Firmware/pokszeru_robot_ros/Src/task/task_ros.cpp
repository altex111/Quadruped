/*
 * task_ros.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#include "task/task_ros.h"

namespace task
{

TaskROS::TaskROS(const char* name, uint16_t stackSize, uint32_t priority,
				EventGroupHandle_t& SchedulerEventGroup,
				MessageBufferHandle_t& ImuToRosMessageQuaternion,
				MessageBufferHandle_t& ImuToRosMessageDebug,
				MessageBufferHandle_t& RosToImuMessageDebug,
				MessageBufferHandle_t& LidarToRosMessage,
				MessageBufferHandle_t& RosToWalkMessage)
:	Task(name, stackSize, priority), node(), SchedulerEventGroup(SchedulerEventGroup),
	ImuToRosMessageQuaternion(ImuToRosMessageQuaternion),
	ImuToRosMessageDebug(ImuToRosMessageDebug),
	RosToImuMessageDebug(RosToImuMessageDebug),
	LidarToRosMessage(LidarToRosMessage),
	RosToWalkMessage(RosToWalkMessage)
{

}

void TaskROS::Process()
{
	/* Check if initialization of member variables were successful */
	if(NULL == ImuToRosMessageQuaternion)
	{
		/* Report error in case of failed initialization */
		return;
	}
	if(NULL == ImuToRosMessageDebug)
	{
		/* Report error in case of failed initialization */
		return;
	}
	if(NULL == RosToImuMessageDebug)
	{
		/* Report error in case of failed initialization */
		return;
	}
	if(NULL == LidarToRosMessage)
	{
		/* Report error in case of failed initialization */
		return;
	}
	if(NULL == RosToWalkMessage)
	{
		/* Report error in case of failed initialization */
		return;
	}

	NVIC_EnableIRQ(USART3_IRQn);

	/* Call init function */
	node.ROSSetup();

	/* Set Wait for ROS bit of the Event Group */
	xEventGroupSetBits(SchedulerEventGroup, SCHEDULEREVENT_WAIT_ROS_INIT_BIT);

	while(true)
	{
		node.Get_schedulerEventBits() = xEventGroupWaitBits(
																SchedulerEventGroup,
																(SCHEDULEREVENT_WIFI_BIT | SCHEDULEREVENT_IMU_BIT | SCHEDULEREVENT_LIDAR_BIT),
																pdTRUE,
																pdFALSE,
																//portMAX_DELAY
																//( 10 / portTICK_PERIOD_MS )
																0
															);

		if(SCHEDULEREVENT_IMU_BIT == (node.Get_schedulerEventBits() & SCHEDULEREVENT_IMU_BIT))
		{
			/* Copy quaternion data from message buffer to local buffer, if the message buffer is not empty */
			if(pdFALSE == xMessageBufferIsEmpty(ImuToRosMessageQuaternion))
			{
				xMessageBufferReceive(ImuToRosMessageQuaternion, (void*)&(node.GetImu_inRequestQuaternion()), FREERTOS_IMUREQUEST_BUFFER_SIZE, 0u);
			}

			/* Copy debug data from message buffer to local buffer, if the message buffer is not empty */
			if(pdFALSE == xMessageBufferIsEmpty(ImuToRosMessageDebug))
			{
				xMessageBufferReceive(ImuToRosMessageDebug, (void*)&(node.GetImu_inRequestDebug()), FREERTOS_IMUREQUEST_BUFFER_SIZE, 0u);

				/* Set the imu_debug_status to DEBUG_REQUEST_STATUS_ARRIVED */
				node.GetImu_debugRequest() = DEBUG_REQUEST_STATUS_ARRIVED;
			}
		}

		if(SCHEDULEREVENT_LIDAR_BIT == (node.Get_schedulerEventBits() & SCHEDULEREVENT_LIDAR_BIT))
		{
			/* Copy Lidar data from message buffer to local buffer, if the message buffer is not empty */
			if(pdFALSE == xMessageBufferIsEmpty(LidarToRosMessage))
			{
				xMessageBufferReceive(LidarToRosMessage, (void*)&(node.GetLidar_LidarScan()), sizeof(sensor_msgs::LaserScan), 0u);
			}
		}

		/* Call ROS loop function */
		node.ROSLoop();

		/* Copy debug data from local buffer to message buffer, if there is a valid data to be sent */
		if(DEBUG_REQUEST_STATUS_REQUESTED == node.GetImu_debugRequest())
		{
			if(IMUREQUEST_DIRECTION_READ == node.GetImu_outRequestDebug().direction)
			{
				xMessageBufferSend(RosToImuMessageDebug, (void*)&(node.GetImu_outRequestDebug()), FREERTOS_IMUREQUEST_FIX_SIZE, 0u);
			}
			else
			{
				xMessageBufferSend(RosToImuMessageDebug, (void*)&(node.GetImu_outRequestDebug()), (FREERTOS_IMUREQUEST_FIX_SIZE + node.GetImu_outRequestDebug().length), 0u);
			}

			/* Set the imu_debug_status to DEBUG_REQUEST_STATUS_IN_PROGRESS */
			node.GetImu_debugRequest() = DEBUG_REQUEST_STATUS_IN_PROGRESS;
		}

		/* Copy command data from local buffer to message buffer, if there is a valid data to be sent */
		if(true == node.GetWalk_commandRequest())
		{
			xMessageBufferSend(RosToWalkMessage, (void*)&(node.GetWalk_Command()), sizeof(WalkCommand), 0u);

			/* Set the walk_command_status to falseS */
			node.GetWalk_commandRequest() = false;
		}

		/* Delay 10 ms */
		// todo test if this breaks everything vTaskDelay( 5 / portTICK_PERIOD_MS );
	}
}

}
