/*
 * moving.h
 *
 *  Created on: 29/07/2012
 *      Author: raul
 */

#ifndef MOVING_H_
#define MOVING_H_

#include <sys/types.h>
#include <reent.h>

#include "stm32f4_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ORIGIN_RADAR 'R'
#define CODE_OBSTACLE_IN_FRONT 'F'


typedef struct {
	portCHAR origin;
	portCHAR code;
	//portCHAR ucData[ 20 ];
} movingMsg;

void init_moving(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin_Left1, uint16_t GPIO_Pin_Left2, uint16_t GPIO_Pin_Right1, uint16_t GPIO_Pin_Right2);

void moving_go_forward();

void moving_go_back();

void moving_turn_left();

void moving_turn_right();

#endif /* MOVING_H_ */
