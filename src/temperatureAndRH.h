/*
 * temperatureAndRH.h
 *
 *  Created on: 17/06/2012
 *      Author: raul
 */

#ifndef TEMPERATUREANDRH_H_
#define TEMPERATUREANDRH_H_

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

/**
 * Initialize all the data structures needed  to read temperature and relative humidity
 * initUsTimer() MUST be called before calling this function.
 *
 * \param  GPIOx
 * \param  GPIO_Pin
 */
void init_temperatureRH(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/*
  * Perform a read of values from temperature and Humidity. While this function is being executed no interruption or context change must be performed.
 *
 * 	\param  	temperatureRH values
 *
*	\return		CODE operation
 */
uint8_t read_values_temperatureRH(temperatureRH * values);


#endif /* TEMPERATUREANDRH_H_ */
