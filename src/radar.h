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
#include "ansi.h"
#include "ustime.h"

/* Definitions -------------------------------------------------------------- */
#define LONG_TIME 0xffff
#define TICKS_TO_WAIT    10

void init_radar(GPIO_TypeDef* GPIOx_trigger, uint16_t GPIO_Pin_trigger, uint32_t RCC_AHB1Periph, uint8_t EXTI_PortSourceGPIOx, GPIO_TypeDef* GPIOx_echo, uint16_t GPIO_Pin_echo);

void trigger(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * Get the distance in centimeters
 *
 * \return the distance in centimers
 */
int32_t getDistance();

/**
 * Performs a read of elapsed time while the specified pin is in high level. It must be called from a ISR
 *
 * \param  	GPIOx
 * 			GPIO_Pin
 */
void processEchoFromISR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**
 * Calculates and set the numeric factor of Speed Sound taking the values of temperature and relative humidity of air.
 *
 * \param	temperature in grades celsius
 * \param	RH porcentage of relative humidity
 */
void calculateSpeedSoundFactor(uint8_t temperature, uint8_t RH);


#endif /* RADAR_H_ */
