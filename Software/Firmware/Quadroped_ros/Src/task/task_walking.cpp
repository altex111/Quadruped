/*
 * task_walking.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Mass�r L�r�nt
 */

#include "task/task_walking.h"

namespace task
{

TaskWalk::TaskWalk(const char* name, uint16_t stackSize, uint32_t priority, MessageBufferHandle_t& RosToWalkMessage)
:	Task(name, stackSize, priority), RosToWalkMessage(RosToWalkMessage), walk()
{}

void TaskWalk::Process()
{
	/* Call init function */
	//walk.Start();

	while(true)
	{
		/* Copy debug data from message buffer to local buffer, if the message buffer is not empty */
		while(pdFALSE == xMessageBufferIsEmpty(RosToWalkMessage))
		{
			xMessageBufferReceive(RosToWalkMessage, (void*)&(walk_Command), sizeof(WalkCommand), 0u);
			walk.StoreCommand(walk_Command);
		}

		/* Call Walk process function */
		walk.Loop();

		/* Delay 20 ms *//* IMPORTANT - changing this delay may cause problems in the code parts controlling the movement */
		vTaskDelay( 20 / portTICK_PERIOD_MS );
	}
}

}
