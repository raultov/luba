/*
 * apc220.c
 *
 *  Created on: 15/02/2013
 *      Author: raul
 */

/*
 * 	Description: set of functions to manage the serial transmitter/receiver apc220 module.
 * 				 USART1 will be set up to manage apc220 module.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

#include "apc220.h"

/* Global variables --------------------------------------------------------- */

#define RX_SIZE  128
#define TX_SIZE  128

static struct ringbuf rx_buf = { .buf = (char[RX_SIZE]) {}, .bufsize = RX_SIZE };
static struct ringbuf tx_buf = { .buf = (char[TX_SIZE]) {}, .bufsize = TX_SIZE };

static volatile struct uart_stats {
    uint32_t    rx_overrun;
    uint32_t    rx_bytes;
    uint32_t    tx_bytes;
} uart_stats;

xSemaphoreHandle xSemaphoreAPC220 = NULL;

/**
 *
 * PRIVATE FUNCTIONS
 *
 */

int apc220_chars_avail(void)
{
    return rx_buf.len;
}

ssize_t apc220_write_r(struct _reent *r, int fd, const void *ptr, size_t len)
{
    const char *c = (const char*) ptr;

    for (int i = 0; i < len; i++) {
        while (!rb_putc(&tx_buf, *c));
        c++;

        // Enable TX empty interrupt
        USART1->CR1 |= USART_CR1_TXEIE;
    }

    return len;
}

ssize_t apc220_read_r(struct _reent *r, int fd, void *ptr, size_t len)
{
    while (!rx_buf.len);

    if (len > rx_buf.len)
        len = rx_buf.len;

    char *c = (char*)ptr;
    for (int i = 0; i < len; i++)
        rb_getc(&rx_buf, c++);

    return len;
}


void apc220_poll_send(const char *ch)
{
    while (*ch) {
        USART1->DR = *ch++ & 0xff;
        while (!(USART1->SR & USART_FLAG_TXE));
        uart_stats.tx_bytes++;
    }
}

/**
 *
 * PUBLIC FUNCTIONS
 *
 */

/**
 * Initialize UART.
 *
 * \param  baudrate  Baudrate
 *
 *  PB6   USART1_TXD
 *  PB7   USART1_RXD
 *
 */
void apc220_init(int baudrate)
{
    // Enable peripheral clocks
    //
    RCC->AHB1ENR |= RCC_AHB1Periph_GPIOB;
    RCC->APB2ENR |= RCC_APB2Periph_USART1;

    // Initialize Serial Port
    //
    GPIO_Init(GPIOB, &(GPIO_InitTypeDef) {
        .GPIO_Pin   = GPIO_Pin_6,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode  = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP
    });

    GPIO_Init(GPIOB, &(GPIO_InitTypeDef) {
        .GPIO_Pin = GPIO_Pin_7,
        .GPIO_Mode = GPIO_Mode_IN,
        .GPIO_PuPd = GPIO_PuPd_UP
    });

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_Init(USART1, &(USART_InitTypeDef) {
        .USART_BaudRate = baudrate,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No ,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
        .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
    });

    NVIC_Init(&(NVIC_InitTypeDef) {
        .NVIC_IRQChannel = USART1_IRQn,
        .NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    });

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // Create the semaphore to protect the tx_buf variable
    xSemaphoreAPC220 = xSemaphoreCreateMutex();
}

void apc220_send_task() {
	char c;
	int result = 0;

	if( xSemaphoreTake( xSemaphoreAPC220, ( portTickType ) 10 ) == pdTRUE ) {
		result = rb_getc(&tx_buf, &c);
		xSemaphoreGive( xSemaphoreAPC220 );
	}

	if (result) {
		while ( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, c);
	}
}

ssize_t apc220_write_str(const void *ptr, size_t len) {
    const char *c = (const char*) ptr;
    int exit;
    int abort = 0;

    for (int i = 0; i < len && !abort; i++) {
    	exit = 0;

    	while (!exit) {
    		if( xSemaphoreTake( xSemaphoreAPC220, ( portTickType ) 10 ) == pdTRUE ) {
    			exit = rb_putc(&tx_buf, *c);
    			xSemaphoreGive( xSemaphoreAPC220 );
    		} else {
    			// Abort writing, it's locked by other task
    			len = -1;
    			abort = pdTRUE;
    			exit = 1;
    		}
    	}

    	if (*c == '\0')
        	break;

        c++;
    }

    return len;
}

ssize_t apc220_read_str(char *ptr) {
	int len = -1;

	// TODO To develop a mechanism that receives a string ended by '\0' and put in on *ptr
	//      It's necessary to use a semaphore in a similar way to the write process

    return len;
}

/**
 *
 * EXTERNAL FUNCTIONS
 *
 */

void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        if (!rb_putc(&rx_buf, USART1->DR)) {
            uart_stats.rx_overrun++;
        } else {
            uart_stats.rx_bytes++;
        }
    }

    if (USART1->SR & USART_SR_TXE) {
        char c;

		if (rb_getc(&tx_buf, &c)) {
			// send a queued byte
				USART1->DR = c;
		} else {
			// nothing to send, disable interrupt
			//
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}

        uart_stats.tx_bytes++;
    }
}
































