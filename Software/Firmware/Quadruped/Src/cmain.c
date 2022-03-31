
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <cmain.h>
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Timer_PostInit()
{
	  LL_TIM_EnableCounter(TIM1);
	  LL_TIM_EnableCounter(TIM2);
	  LL_TIM_EnableCounter(TIM3);
	  LL_TIM_EnableCounter(TIM4);
}

void ADC_PostInit()
{
	const uint32_t LOOP_CYCLE_TIME = 4;
	uint32_t wait_loop_index;

	LL_ADC_DisableDeepPowerDown(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);
	wait_loop_index = (LL_ADC_DELAY_INTERNAL_REGUL_STAB_US*SystemCoreClock) / (LOOP_CYCLE_TIME*1000000);
	while (wait_loop_index--);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while (LL_ADC_IsCalibrationOnGoing(ADC1));

	LL_ADC_ClearFlag_ADRDY(ADC1);
	LL_ADC_Enable(ADC1);
	while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
}

void USART3_IRQCallback(){}
void UART4_IRQCallback(){}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
void Init(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  Timer_PostInit();
  ADC_PostInit();
  /* USER CODE END 2 */

}
static void LL_Init(void)
{
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
#define EXTERNAL_CLOCK 0
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

#if EXTERNAL_CLOCK == 1
  LL_RCC_HSE_Enable();
#else
  LL_RCC_HSI_Enable();
#endif

   /* Wait till HSE is ready */
#if EXTERNAL_CLOCK == 1
  while(LL_RCC_HSE_IsReady() != 1);
#else
  while(LL_RCC_HSI_IsReady() != 1);
#endif


#if EXTERNAL_CLOCK == 1
  uint32_t clockSrc = LL_RCC_PLLSOURCE_HSE;
#else
  uint32_t clockSrc = LL_RCC_PLLSOURCE_HSI;
#endif

  LL_RCC_PLL_ConfigDomain_SYS(clockSrc, LL_RCC_PLLM_DIV_1,
#if EXTERNAL_CLOCK == 1
		  20
#else
		  10
#endif
		  , LL_RCC_PLLR_DIV_2);

  LL_RCC_PLL_EnableDomain_SYS();

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_PLLSAI1_ConfigDomain_ADC(clockSrc, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLSAI1R_DIV_2);

  LL_RCC_PLLSAI1_EnableDomain_ADC();

  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {
    
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(80000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(80000000);

  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC1_2_IRQn interrupt configuration */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USART3_IRQn interrupt configuration */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);
  /* UART4_IRQn interrupt configuration */
  NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UART4_IRQn);
  /* TIM4_IRQn interrupt configuration */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
