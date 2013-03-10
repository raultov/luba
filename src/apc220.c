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

/* In the case of receiving data, we use a double buffer,
 * if one is blocked by the reader task then the other one will be available
 * and could be used by the ISR
 */
static struct ringbuf rx_buf1 = { .buf = (char[RX_SIZE]) {}, .bufsize = RX_SIZE };
static struct ringbuf rx_buf2 = { .buf = (char[RX_SIZE]) {}, .bufsize = RX_SIZE };
static struct ringbuf tx_buf = { .buf = (char[TX_SIZE]) {}, .bufsize = TX_SIZE };

static char buffer[RX_SIZE];
static int buffer_index = 0;

static volatile struct uart_stats {
    uint32_t    rx_buff1_busy; // Amount of times rx_buff1 was busy and cause of that rx_buff2 has been used
    uint32_t    rx_bytes;
    uint32_t    tx_bytes;
} uart_stats;

xSemaphoreHandle xSemaphoreTx = NULL;
xSemaphoreHandle xSemaphoreRx = NULL;

/**
 *
 * PRIVATE FUNCTIONS
 *
 */

int apc220_chars_avail(void)
{
    return rx_buf1.len;
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
    while (!rx_buf1.len);

    if (len > rx_buf1.len)
        len = rx_buf1.len;

    char *c = (char*)ptr;
    for (int i = 0; i < len; i++)
        rb_getc(&rx_buf1, c++);

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
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);

    // Create semaphores to protect the rx_buf and tx_buf variables
    xSemaphoreTx = xSemaphoreCreateMutex();
    xSemaphoreRx = xSemaphoreCreateMutex();

    // initialize stats
    uart_stats.rx_bytes = 0;
    uart_stats.tx_bytes = 0;
    uart_stats.rx_buff1_busy = 0;
}

void apc220_send_task() {
	char c;
	int result = 0;

	if( xSemaphoreTake( xSemaphoreTx, ( portTickType ) 10 ) == pdTRUE ) {
		result = rb_getc(&tx_buf, &c);
		xSemaphoreGive( xSemaphoreTx );
	}

	if (result) {
		while ( !(USART1->SR & 0x00000040) );
		USART_SendData(USART1, c);
		uart_stats.tx_bytes++;
	}
}

ssize_t apc220_write_str(const void *ptr, size_t len) {
    const char *c = (const char*) ptr;
    int exit;
    int abort = 0;

    for (int i = 0; i < len && !abort; i++) {
    	exit = 0;

    	while (!exit) {
    		if( xSemaphoreTake( xSemaphoreTx, ( portTickType ) 10 ) == pdTRUE ) {
    			exit = rb_putc(&tx_buf, *c);
    			xSemaphoreGive( xSemaphoreTx );
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

/**
 * @brief This function collects data from RX and put it on the variable ptr
 * @note *ptr must be allocated previously
 *
 * @param  *ptr the variable to be filled when a message is available
 *
 * @retval  -1 when no message is available
 * 			!= -1 the length of the message copied into *ptr
 */

ssize_t apc220_read_str(char *ptr) {
	int len = -1;
	int result = 0;
	char c;

	if ( xSemaphoreRx != NULL ) {
		if( xSemaphoreTake( xSemaphoreRx, 0 ) == pdTRUE ) {

			result = rb_getc(&rx_buf1, &c);
			if (result != 0) {
				buffer[buffer_index++] = c;
			}

			xSemaphoreGive( xSemaphoreRx );

			// We don't need to protect rx_buf2 because at this point rx_buf1 is available so
			// the ISR will attempt to write into rx_buf1 instead of rx_buf2
			result = rb_getc(&rx_buf2, &c);

			if (result != 0) {
				buffer[buffer_index++] = c;
			}
		}
	}

	if ((buffer_index > 0) && (buffer[buffer_index-1] == '\0')) {
		len = buffer_index-1;

		for (int i=0 ;i < len ; i++) {
			ptr[i] = buffer[i];
		}

		buffer_index = 0;
	}

    return len;
}

/**
 *
 * EXTERNAL FUNCTIONS
 *
 */

void USART1_IRQHandler(void) {

	static signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if ( USART_GetITStatus(USART1, USART_IT_RXNE) ) {

		if ( xSemaphoreRx != NULL ) {
			if (xQueueReceiveFromISR( xSemaphoreRx, NULL, NULL)) {
				// The first rx buffer is available
				xSemaphoreGiveFromISR( xSemaphoreRx, &xHigherPriorityTaskWoken );
				rb_putc(&rx_buf1, USART1->DR);
			} else {
				// The first rx buffer is locked by the reader task
				// so we have to use the second buffer which is of course available
				rb_putc(&rx_buf2, USART1->DR);
				uart_stats.rx_buff1_busy++;
			}
		}

		uart_stats.rx_bytes++;
    }
}
































