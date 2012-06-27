/*
 * temperatureAndHR.c
 *
 *  Created on: 17/06/2012
 *      Author: raul
 */

/*
 * 	Description: set of functions to manage the sensor DHT11 which measures temperature and humidity of air
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "temperatureAndRH.h"

/* Global variables --------------------------------------------------------- */
GPIO_TypeDef* GPIOx_temperatureRH;
uint16_t GPIO_Pin_temperatureRH;
GPIO_InitTypeDef  GPIO_InitStructure_temperatureRH;

/* Private functions ---------------------------------------------------------*/

void input_mode_temperatureRH() {
	GPIO_InitStructure_temperatureRH.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOx_temperatureRH, &GPIO_InitStructure_temperatureRH);
}

void output_mode_temperatureRH() {
	GPIO_InitStructure_temperatureRH.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOx_temperatureRH, &GPIO_InitStructure_temperatureRH);
}

/* Functions ---------------------------------------------------------------- */

void init_temperatureRH(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	GPIOx_temperatureRH = GPIOx;
	GPIO_Pin_temperatureRH = GPIO_Pin;

	GPIO_InitStructure_temperatureRH.GPIO_Pin = GPIO_Pin_temperatureRH;
	GPIO_InitStructure_temperatureRH.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_temperatureRH.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_temperatureRH.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_temperatureRH.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOx_temperatureRH, &GPIO_InitStructure_temperatureRH);

	delayUs(1000000);  // Wait 1s recommended delay before accessing sensor
}

/*
  * Perform a read of values from temperature and Humidity. While this function is being executed no interruption or context change must be performed.
 *
 * 	\param  	temperatureRH values
 *
*	\return		CODE operation
 */
 uint8_t read_values_temperatureRH(temperatureRH * values) {

        // BUFFER TO RECEIVE
        uint8_t bits[5];
        uint8_t cnt = 7;
        uint8_t idx = 0;

        // EMPTY BUFFER
        for (int i=0; i< 5; i++) bits[i] = 0;

        // REQUEST SAMPLE
        output_mode_temperatureRH();
        GPIO_ResetBits(GPIOx_temperatureRH, GPIO_Pin_temperatureRH);
        delayMs(18);
        GPIO_SetBits(GPIOx_temperatureRH, GPIO_Pin_temperatureRH);
        delayUs(40);
        input_mode_temperatureRH();

        // ACKNOWLEDGE or TIMEOUT
        unsigned int loopCnt = 10000;
        while (GPIO_ReadInputDataBit(GPIOx_temperatureRH, GPIO_Pin_temperatureRH) == Bit_RESET) {
        	if (loopCnt-- == 0) {
        		return DHT_ERROR_TIMEOUT;
        	}
        }

        loopCnt = 10000;
        while (GPIO_ReadInputDataBit(GPIOx_temperatureRH, GPIO_Pin_temperatureRH) == Bit_SET) {
        	if (loopCnt-- == 0) {
        		return DHT_ERROR_TIMEOUT;
        	}
        }

        // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
        for (int i=0; i<40; i++) {

        	loopCnt = 10000;
            while (GPIO_ReadInputDataBit(GPIOx_temperatureRH, GPIO_Pin_temperatureRH) == Bit_RESET) {
            	if (loopCnt-- == 0) {
            		return DHT_ERROR_TIMEOUT;
            	}
            }

            uint64_t t = getUsTime();

            loopCnt = 10000;
            while (GPIO_ReadInputDataBit(GPIOx_temperatureRH, GPIO_Pin_temperatureRH) == Bit_SET) {
            	if (loopCnt-- == 0) {
            		return DHT_ERROR_TIMEOUT;
            	}
            }

            if ((getUsTime() - t) > 40) {
            	bits[idx] |= (1 << cnt);
            }

            if (cnt == 0)  { // next byte?
                    cnt = 7;    // restart at MSB
                    idx++;      // next byte!
            } else {
            	cnt--;
            }
        }

        // WRITE TO RIGHT VARS
        // as bits[1] and bits[3] are allways zero they are omitted in formulas.
        values->temperature = bits[0];
        values->RH = bits[2];

        uint8_t sum = bits[0] + bits[2];

        if (bits[4] != sum) {
        	return DHT_ERROR_CHECKSUM;
        }

        return DHT_OK;
}



