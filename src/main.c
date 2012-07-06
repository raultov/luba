/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4_discovery.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ansi.h"
#include "ustime.h"

#include "radar.h"
#include "temperatureAndRH.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

xTaskHandle radarHandle;
xTaskHandle ledsHandle;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct task_param {
    char *name;
    int   interval;
};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static void radar_task(void *pvParameters) {

	for ( ; ; ) {

		trigger(GPIOD, GPIO_Pin_15);

		int32_t distance = getDistance();

		if ((distance >= 10) && (distance < 20)) {
			GPIO_SetBits(GPIOD, GPIO_Pin_13);
		} else {
        	GPIO_ResetBits(GPIOD, GPIO_Pin_13);
        }
	}
}

static void leds_task(void *pvParameters) {

	while (1) {
		/* PD12 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		delayUs(1000000);

		/* PD14 to be toggled */
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		delayUs(1000000);

		GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_14);
		delayUs(1000000);
		/* Insert delay */
		//Delay(p->interval);
	}
}

static void temperatureRH_task (void *pvParameters) {

	temperatureRH * values;
	values = malloc (sizeof(struct TEMPERATURE_RH));

	while (1) {

		// We read values of temperature and relative humidity every 30 seconds
		delayUs(30000000);

		vTaskSuspend(radarHandle);
		vTaskSuspend(ledsHandle);

		uint8_t ret = read_values_temperatureRH(values);

		if (ret == 0) {
			calculateSpeedSoundFactor(values->temperature, values->RH);
		}

		vTaskResume(ledsHandle);
		vTaskResume(radarHandle);
	}
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */


	// FreeRTOS assumes 4 preemption- and 0 subpriority-bits
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// Enable GPIOA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	initUsTimer();

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	initRadar(GPIOD, GPIO_Pin_15, RCC_AHB1Periph_GPIOA, EXTI_PortSourceGPIOA, GPIOA, GPIO_Pin_0);
	init_temperatureRH(GPIOA, GPIO_Pin_3);

	struct task_param *p;

	p = malloc(sizeof(struct task_param));
	p->name = malloc(16);
	p->interval = 0xFFFFFF;
	sprintf(p->name, "FPU_%d", 0x0FFFFF);

	xTaskCreate(radar_task, (int8_t*)p->name, 1024, NULL, tskIDLE_PRIORITY, &radarHandle);

	xTaskCreate(leds_task, (int8_t*)p->name, 1024, p, tskIDLE_PRIORITY, &ledsHandle);

	xTaskCreate(temperatureRH_task, (int8_t*)p->name, 1024, NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	return 0;
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
