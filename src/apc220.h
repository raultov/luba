/*
 * apc220.h
 *
 *  Created on: 15/02/2013
 *      Author: raul
 */

#ifndef APC220_H
#define APC220_H

#include <sys/types.h>
#include <reent.h>

#include "FreeRTOS.h"
#include "ustime.h"
#include "stm32f4xx.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "ringbuf.h"

extern void  apc220_init(int baudrate);
void apc220_send_task();
ssize_t apc220_write_str(const void *ptr, size_t len);
ssize_t apc220_read_str(char *ptr);
/*
extern void  apc220_poll_send(const char *s);
extern int   apc220_chars_avail(void);

extern ssize_t apc220_write_r(struct _reent *r, int fd, const void *ptr, size_t len);
extern ssize_t apc220_read_r(struct _reent *r, int fd, void *ptr, size_t len);
*/
#endif
