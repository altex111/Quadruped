/*
 * task_lidar.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#include "task/task_lidar.h"

namespace task
{

TaskLIDAR::TaskLIDAR(const char* name, uint16_t stackSize, uint32_t priority, EventGroupHandle_t& SchedulerEventGroup, MessageBufferHandle_t& LidarToRosMessage)
:	Task(name, stackSize, priority), lidar(UART4), SchedulerEventGroup(SchedulerEventGroup), LidarToRosMessage(LidarToRosMessage)
{}

void TaskLIDAR::Process()
{
	if(NULL == LidarToRosMessage)
	{
		/* Report error in case of failed initialization */
		return;
	}

	/* Wait for ROS init, processing data is pointless without ROS */
	xEventGroupWaitBits(
						SchedulerEventGroup,
						SCHEDULEREVENT_WAIT_ROS_INIT_BIT,
						pdFALSE,
						pdFALSE,
						portMAX_DELAY
						);

	NVIC_EnableIRQ(UART4_IRQn);

	/* Call init function */
	lidar.Start();

	while(true)
	{
		/* Call LDIAR process function */
		if(lidar.Process())
		{
			/* Copy the LIDAR data to the message buffer for the ROS task */
			xMessageBufferSend(LidarToRosMessage, (void*)&(lidar.GetLidar_LaserScan()), sizeof(sensor_msgs::LaserScan), 0u);

			/* Set IMU bit of the Event Group  */
			xEventGroupSetBits(SchedulerEventGroup, SCHEDULEREVENT_LIDAR_BIT);
		}

		/* Delay 50 ms */
		vTaskDelay( 20 / portTICK_PERIOD_MS );
	}
}

}
