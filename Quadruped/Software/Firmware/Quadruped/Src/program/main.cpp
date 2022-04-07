#include "program/walkprogram.h"
#include "stm32l4xx_it.h"

quad::WalkProgram g_program;

extern "C"
{
void UART4InterruptCallback()
{
	g_program.getPCComm().InterruptHandler();
}
}

int main()
{
	Init();

	g_program.Start();

	uint8_t ledVar = 0;
	uint32_t counter = 0;
	while(true)
	{
		if (TryUpdate())
		{
			LL_ADC_REG_StartConversion(ADC1);
			quad::IO::Update();

			g_program.Loop();

			if (++counter == 50)
			{
				counter = 0;
				ledVar++;
				quad::IO::led3.SetTo(ledVar & 1);
				/*quad::IO::led1.SetTo((ledVar & 3) == 0);
				quad::IO::led2.SetTo(ledVar & 1);
				quad::IO::led3.SetTo((ledVar & 3) == 2);*/
			}
		}
	}
}
