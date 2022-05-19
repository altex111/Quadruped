#include "stm32l4xx_it.h"
#include "perif/io.h"

#include "FreeRTOS.h"
#include "task.h"

#include "task/task_ros.h"
#include "task/task_imu.h"
#include "task/task_lidar.h"
#include "task/task_walking.h"
#include "task/task_adc.h"
#include "program/common.h"
/* todo delete*/
#include "message_buffer.h"
#include "event_groups.h"
/* todo delete*/

uint32_t memoryAllocated = 0u;

EventGroupHandle_t SchedulerEventGroup;

MessageBufferHandle_t ImuToRosMessageQuaternion;
MessageBufferHandle_t ImuToRosMessageDebug;
MessageBufferHandle_t RosToImuMessageDebug;

MessageBufferHandle_t LidarToRosMessage;

MessageBufferHandle_t RosToWalkMessage;

/* FreeRTOS task handlers */
task::TaskROS* taskROS;
task::TaskIMU* taskIMU;
task::TaskLIDAR* taskLIDAR;
task::TaskWalk* taskWalk;
task::TaskADC* taskADC;

extern "C"
{
void UART4InterruptCallback()
{
	if(NULL != taskLIDAR)
	{
		taskLIDAR->getLIDAR().getUART().InterruptHandler();
	}
}
void USART1InterruptCallback()
{
	if(NULL != taskIMU)
	{
		taskIMU->getIMU().getUART().InterruptHandler();
	}
}
void USART3InterruptCallback()
{
	if(NULL != taskROS)
	{
		taskROS->getROSNode().getESP().getUART().InterruptHandler();

		if(SCHEDULEREVENT_WIFI_BIT != (xEventGroupGetBitsFromISR(SchedulerEventGroup) & SCHEDULEREVENT_WIFI_BIT))
		{
			BaseType_t higherPriorityTaskWoken = pdFALSE;
			BaseType_t result = pdFALSE;

			/* Set Wifi bit of the Event Group */
			result = xEventGroupSetBitsFromISR(SchedulerEventGroup, SCHEDULEREVENT_WIFI_BIT, &higherPriorityTaskWoken);

			if(pdFALSE != result)
			{
				portYIELD_FROM_ISR(higherPriorityTaskWoken);
			}
		}
	}
}
}

int main()
{
	Init();

	SchedulerEventGroup = xEventGroupCreate();
	if(NULL == SchedulerEventGroup)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}

	/* Creating message buffers */
	ImuToRosMessageQuaternion = xMessageBufferCreate(FREERTOS_MESSAGEBUFFER_LENGTH_SMALL);
	if(NULL == ImuToRosMessageQuaternion)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}
	ImuToRosMessageDebug = xMessageBufferCreate(FREERTOS_MESSAGEBUFFER_LENGTH_SMALL);
	if(NULL == ImuToRosMessageDebug)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}
	RosToImuMessageDebug = xMessageBufferCreate(FREERTOS_MESSAGEBUFFER_LENGTH_SMALL);
	if(NULL == RosToImuMessageDebug)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}
	LidarToRosMessage = xMessageBufferCreate(FREERTOS_MESSAGEBUFFER_LENGTH_BIG);
	if(NULL == LidarToRosMessage)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}
	RosToWalkMessage = xMessageBufferCreate(FREERTOS_MESSAGEBUFFER_LENGTH_SMALL);
	if(NULL == RosToWalkMessage)
	{
		/* Report error in case of failed initialization */
		/* todo */
		return 0;
	}

	/* FreeRTOS task handlers */
	taskROS = new task::TaskROS("TaskROS", 1024, 2, SchedulerEventGroup, ImuToRosMessageQuaternion, ImuToRosMessageDebug, RosToImuMessageDebug, LidarToRosMessage, RosToWalkMessage);
	taskIMU = new task::TaskIMU ("TaskIMU", 512, 4, SchedulerEventGroup, ImuToRosMessageQuaternion, ImuToRosMessageDebug, RosToImuMessageDebug);
	taskLIDAR = new task::TaskLIDAR("TaskLIDAR", 512, 4, SchedulerEventGroup, LidarToRosMessage);
	taskWalk = new task::TaskWalk("TaskWalk", 512, 3, RosToWalkMessage);
	taskADC = new task::TaskADC("TaskADC", 128, 3);

	/* Creating tasks */
	taskROS->Create();
	taskIMU->Create();
	taskLIDAR->Create();
	taskWalk->Create();
	taskADC->Create();

	quadroped::IO::led3.Set();

	/* Start scheduler */
	task::Task::StartScheduler();

	while(true)
	{
		/* Tasks should not return, report error */
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	while(1)
	{

	}
}
