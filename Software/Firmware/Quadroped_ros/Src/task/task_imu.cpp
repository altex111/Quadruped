/*
 * task_imu.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#include "task/task_imu.h"

namespace task
{

TaskIMU::TaskIMU(const char* name, uint16_t stackSize, uint32_t priority, EventGroupHandle_t& SchedulerEventGroup, MessageBufferHandle_t& ImuToRosMessageQuaternion, MessageBufferHandle_t& ImuToRosMessageDebug, MessageBufferHandle_t& RosToImuMessageDebug)
:	Task(name, stackSize, priority), imu(USART1), SchedulerEventGroup(SchedulerEventGroup), ImuToRosMessageQuaternion(ImuToRosMessageQuaternion), ImuToRosMessageDebug(ImuToRosMessageDebug), RosToImuMessageDebug(RosToImuMessageDebug)
{

}

void TaskIMU::Process()
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

	NVIC_EnableIRQ(USART1_IRQn);

	/* Call init function */
	while(false == imu.Start())
	{
		/* Set imu calibration fail bit of the Event Group  */
		xEventGroupSetBits(SchedulerEventGroup, SCHEDULEREVENT_IMU_CALIBRFAIL_BIT);
	}

	/* Set IMU_CALIBRATION bit of the Event Group  */
	xEventGroupSetBits(SchedulerEventGroup, SCHEDULEREVENT_IMU_CALIBRATION_BIT);

	/* Wait for ROS init, processing data is pointless without ROS */
	xEventGroupWaitBits(
						SchedulerEventGroup,
						SCHEDULEREVENT_WAIT_ROS_INIT_BIT,
						pdFALSE,
						pdFALSE,
						portMAX_DELAY
						);

	while(true)
	{
		/* Copy debug data from message buffer to local buffer, if the message buffer is not empty */
		if(pdFALSE == xMessageBufferIsEmpty(RosToImuMessageDebug))
		{
			xMessageBufferReceive(RosToImuMessageDebug, (void*)&(imu.GetImu_inRequestDebug()), FREERTOS_IMUREQUEST_BUFFER_SIZE, 0u);

			/* Set debug request flag */
			imu.SetImu_debugRequest();
		}

		/* Call IMU process function */
		imu.Process();

		/* Copy quaternion data from local buffer to message buffer (there should always be valid data to send) */
		xMessageBufferSend(ImuToRosMessageQuaternion, (void*)&(imu.GetImu_outRequestQuaternion()), (FREERTOS_IMUREQUEST_FIX_SIZE + imu.GetImu_outRequestQuaternion().length), 0u);


		/* Copy debug data from local buffer to message buffer, if there is a valid data to be sent */
		if(imu.GetImu_debugRequest())
		{
			xMessageBufferSend(ImuToRosMessageDebug, (void*)&(imu.GetImu_outRequestDebug()), (FREERTOS_IMUREQUEST_FIX_SIZE + imu.GetImu_outRequestDebug().length), 0u);

			/* Clear debug request flag */
			imu.ClearImu_debugRequest();
		}

		/* Set IMU bit of the Event Group  */
		xEventGroupSetBits(SchedulerEventGroup, SCHEDULEREVENT_IMU_BIT);

		/* Delay 10 ms */
		//todo test vTaskDelay( 10 / portTICK_PERIOD_MS );
		vTaskDelay( 20 / portTICK_PERIOD_MS );
	}
}

}
