#ifndef US_TIME_H
#define US_TIME_H

#include <stdint.h>

// Grupo de funciones que usa el timer TIM7 (16 bits)
void	delayUs(unsigned long us);
void	delayMs(unsigned long ms);
uint64_t	getUsTime(void);
void	initUsTimer(void);

// Grupo de funciones que usa el timer TIM2 (32 bits)
void 	initCounterUs(void);
void	resetCounterUs(void);
uint32_t	getCounterUs(void);
void 	delayCounterUs(unsigned long us);
void 	stopCounterUs();

#endif
