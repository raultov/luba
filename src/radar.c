/*
 * radar.c
 *
 *  Created on: 12/06/2012
 *      Author: raul
 */

/*
 * 	Description: Set of functions to manage the peripheric device HC-SR04 which consists of a ultrasonic sensor capable of calculate the distance
 * 				 from the sensor to the next solid obstacle.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "radar.h"

/* Global variables --------------------------------------------------------- */
uint64_t t0 = 0;
uint64_t delta = 0;
xSemaphoreHandle xSemaphore = NULL;
int32_t distance = -1;


/* Functions ---------------------------------------------------------------- */

void initRadar(GPIO_TypeDef* GPIOx_trigger, uint16_t GPIO_Pin_trigger, uint32_t RCC_AHB1Periph, uint8_t EXTI_PortSourceGPIOx, GPIO_TypeDef* GPIOx_echo, uint16_t GPIO_Pin_echo) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_trigger;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOx_trigger, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_echo;
	GPIO_Init(GPIOx_echo, &GPIO_InitStructure);

	/* Connect EXTI Line0 to pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOx, EXTI_PinSource0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_GenerateSWInterrupt(EXTI_Line0);

	vSemaphoreCreateBinary( xSemaphore );
}

void trigger(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {

	GPIO_ResetBits(GPIOx, GPIO_Pin);
	delayUs(2);
	GPIO_SetBits(GPIOx, GPIO_Pin);
	delayUs(10);
	GPIO_ResetBits(GPIOx, GPIO_Pin);

	delayUs(30000);

}

/**
 * Get the distance in centimeters
 *
 * \return the distance in centimers
 */
int32_t getDistance() {
	//int32_t distance = -1;

	// See if we can obtain the semaphore.  If the semaphore is not available
	// wait 10 ticks to see if it becomes free.
	/*if ( xSemaphoreTake( xSemaphore, ( portTickType ) 10 ) == pdTRUE ) {

		distance = delta / 58;
	}*/

	/*
	if (calculating != 1) {
		distance = delta / 58;
	}
*/
	return distance;
}

/**
 * Perform a read of elapsed time while the specified pin is in high level. It must be called from a ISR
 *
 * \param  	GPIOx
 * 			GPIO_Pin
 */
void processEchoFromISR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {

	uint8_t state = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);

	if (state == Bit_RESET) {
		uint64_t t;
		t = getUsTime();

		delta = t - t0;

		distance = delta / 58;


//		if (xSemaphoreGiveFromISR( xSemaphore, pdFALSE ) != pdTRUE ) {
//	           // We would not expect this call to fail because we must have
//	            // obtained the semaphore to get here.
//		}

	} else {
		t0 = getUsTime();
	}

}




















