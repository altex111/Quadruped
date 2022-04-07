/*
 * task_adc.cpp
 *
 *  Created on: 2020. okt. 09.
 *      Author: Massár Lóránt
 */

#include "task/task_adc.h"
#include "stm32l4xx_it.h"

namespace task
{

TaskADC::TaskADC(const char* name, uint16_t stackSize, uint32_t priority)
:	Task(name, stackSize, priority)
{}

void TaskADC::Process()
{
	/* Call init function */

	while(true)
	{
		/* Call ADC start conversion function */
		LL_ADC_REG_StartConversion(ADC1);

		/* Delay 20 ms */
		vTaskDelay( 20 / portTICK_PERIOD_MS );
	}
}

}
