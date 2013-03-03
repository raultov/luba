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

// 58 is the amount of microseconds spent by the sound wave to go over 2 cm. The speed of sound used to determine this factor is 340 m/s
uint64_t factorSoundSpeed = 58;


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

	xSemaphore = xSemaphoreCreateMutex( );
}

void trigger(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {

	GPIO_ResetBits(GPIOx, GPIO_Pin);
	delayUs(2);
	GPIO_SetBits(GPIOx, GPIO_Pin);
	delayUs(10);
	GPIO_ResetBits(GPIOx, GPIO_Pin);

	delayUs(30000);

}

int32_t getDistance() {
	return distance;
}

void processEchoFromISR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {

	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	uint8_t state = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin);

	if (state == Bit_RESET) {
		uint64_t t;
		t = getUsTime();

		delta = t - t0;

		if ( xSemaphore != NULL ) {
			if (xQueueReceiveFromISR( xSemaphore, NULL, NULL)) {
				distance = delta / factorSoundSpeed;

				xHigherPriorityTaskWoken = pdFALSE;
				xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
			}
		}
	} else {
		t0 = getUsTime();
	}

}

void calculateSpeedSoundFactor(uint8_t temperature, uint8_t RH) {
	float n = 2.0f;
	float t = (float) temperature;
	float d = (0.0000606f * t) + 0.03313f;

	if( xSemaphore != NULL ) {
		if( xSemaphoreTake( xSemaphore, 0 ) == pdTRUE ) {
			factorSoundSpeed = (int) (n / d);

			xSemaphoreGive( xSemaphore );
		}
	}
}




















