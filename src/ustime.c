#include "ustime.h"
#include <FreeRTOS.h>
#include "stm32f4xx.h"

/**
 * Get time count in microseconds.
 *
 * \note
 *   this function must be called at least
 *   once every 65ms to work correctly.
 *
 */


uint64_t getUsTime()
{
    static uint16_t t0;
    static uint64_t tickcount;

    vPortEnterCritical();

    int t = TIM7->CNT;
    if (t < t0)
        t += 0x10000;

    tickcount += t - t0;
    t0 = t;

    vPortExitCritical();

    return tickcount;
}


/**
 * Perform a microsecond delay
 *
 * \param  us  number of microseconds to wait.
 * \note   The actual delay will last between us and (us-1) microseconds.
 *         To wait _at_least_ 1 us, you should use delay_us(2).
 */
void delayUs(unsigned long us)
{
    uint16_t  t0 = TIM7->CNT;
    for (;;) {
        int  t = TIM7->CNT;
        if (t < t0)
            t += 0x10000;

        if (us < t - t0)
            return;

        us -= t - t0;
        t0  = t;
    }
}


/**
 * Performs a millisecond delay
 *
 * \param  ms  number of milliseconds to wait.
 */
void delayMs(unsigned long ms)
{
    delayUs(ms * 1000);
}

void resetCounterUs() {
	initCounterUs();
}

uint32_t getCounterUs() {
	return TIM2->CNT;
}

/*
 * No usar un retraso superior a 4294 segundos
 */

void delayCounterUs(unsigned long us) {
	vPortEnterCritical();
	uint32_t t0 = TIM2->CNT;
	vPortExitCritical();

	uint32_t acc = 0;

	for (;;) {
		vPortEnterCritical();
		uint32_t t = TIM2->CNT;
		vPortExitCritical();

		if (t < t0) {
			acc = 0x100000000 - t0;
			t0 = 0;
		}

		uint32_t delta = t - t0 + acc;

		if (us < delta) {
			return;
		}
	}
}

void stopCounterUs() {
	//TIM_Cmd(TIM2, DISABLE);
}

/**
 * Set up TIM2 as a 32bit, microsecond-timer.
 *
 */
void initUsTimer()
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    RCC->APB1ENR |= RCC_APB1Periph_TIM7;
    TIM7->PSC = (RCC_Clocks.PCLK2_Frequency / 1000000) - 1;
    TIM7->ARR = 0xFFFF;
    TIM7->CR1 = TIM_CR1_CEN;
}

void initCounterUs() {
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    RCC->APB1ENR |= RCC_APB1Periph_TIM2;
    TIM2->PSC = (RCC_Clocks.PCLK2_Frequency / 1000000) - 1; // microsegundos
    TIM2->ARR = 0xFFFFFFFF; // 2^32
    TIM2->CR1 = TIM_CR1_CEN;

    //TIM_Cmd(TIM2, ENABLE);
}





