/*
 * moving.c
 *
 *  Created on: 29/07/2012
 *      Author: raul
 */

/*
 * 	Description: Set of functions to manage wheels
 *
 */

#include "moving.h"

/* Global variables --------------------------------------------------------- */
GPIO_TypeDef* GPIOx_Moving;
uint16_t GPIO_Pin_L1;
uint16_t GPIO_Pin_L2;
uint16_t GPIO_Pin_R1;
uint16_t GPIO_Pin_R2;
GPIO_InitTypeDef  GPIO_InitStructure_Moving;

//static movingMsg movingMsgRcv;

/**
 * @brief This function initializes the resources associated to the moving of wheels
 *
 * @param  GPIOx bus, GPIO_Pin_Leftx left wheel, GPIO_Pin_Rightx right wheel
 *
 */
void init_moving(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin_Left1, uint16_t GPIO_Pin_Left2, uint16_t GPIO_Pin_Right1, uint16_t GPIO_Pin_Right2) {
	GPIOx_Moving = GPIOx;
	GPIO_Pin_L1 = GPIO_Pin_Left1;
	GPIO_Pin_L2 = GPIO_Pin_Left2;
	GPIO_Pin_R1 = GPIO_Pin_Right1;
	GPIO_Pin_R2 = GPIO_Pin_Right2;

	GPIO_InitStructure_Moving.GPIO_Pin = GPIO_Pin_Left1 | GPIO_Pin_Left2 | GPIO_Pin_Right1 | GPIO_Pin_Right2;
	GPIO_InitStructure_Moving.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_Moving.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_Moving.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure_Moving.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOx, &GPIO_InitStructure_Moving);
}

void moving_go_forward() {
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_L1);
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_L2);

	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_R1);
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_R2);
}

void moving_turn_left() {
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_L1);
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_L2);

	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_R1);
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_R2);
}

void moving_turn_right() {
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_L1);
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_L2);

	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_R1);
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_R2);
}

void moving_go_back() {
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_L2);
	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_L1);

	GPIO_ResetBits(GPIOx_Moving, GPIO_Pin_R2);
	GPIO_SetBits(GPIOx_Moving, GPIO_Pin_R1);
}





























