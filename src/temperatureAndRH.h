/*
 * temperatureAndHR.h
 *
 *  Created on: 17/06/2012
 *      Author: raul
 */

#ifndef TEMPERATUREANDHR_H_
#define TEMPERATUREANDHR_H_

#include "stm32f4_discovery.h"

#include "FreeRTOS.h"
#include "ansi.h"
#include "ustime.h"

/* DEFINES *****************************************************************************/
#define DHT_OK                               0
#define DHT_ERROR_CHECKSUM   -1
#define DHT_ERROR_TIMEOUT    -2

/*
 * Defines the temperature and RH of air
 */
typedef struct TEMPERATURE_RH
{
	uint8_t temperature;
	uint8_t RH;
} temperatureRH;

void init_temperatureRH(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

uint8_t read_values_temperatureRH(temperatureRH * values);


#endif /* TEMPERATUREANDHR_H_ */
