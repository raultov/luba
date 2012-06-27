/*
 * radar.h
 *
 *  Created on: 12/06/2012
 *      Author: raul
 */

#ifndef RADAR_H_
#define RADAR_H_

#include "stm32f4_discovery.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "ansi.h"
#include "ustime.h"

/* Definitions -------------------------------------------------------------- */
#define LONG_TIME 0xffff
#define TICKS_TO_WAIT    10

void initRadar(GPIO_TypeDef* GPIOx_trigger, uint16_t GPIO_Pin_trigger, uint32_t RCC_AHB1Periph, uint8_t EXTI_PortSourceGPIOx, GPIO_TypeDef* GPIOx_echo, uint16_t GPIO_Pin_echo);

void trigger(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

int32_t getDistance();

void processEchoFromISR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


#endif /* RADAR_H_ */
