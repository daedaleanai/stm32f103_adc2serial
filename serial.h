#pragma once

#include "stdarg.h"
#include "stddef.h"

#include "stm32f103_md.h"

struct Ringbuffer {
	uint16_t head;     // writes happen here
	uint16_t tail;     // reads happen here
	uint8_t  buf[512]; // size must be power of two
};

inline uint16_t ringbuffer_avail(struct Ringbuffer* rb) { return rb->head - rb->tail; }                        // 0..size -1
inline uint16_t ringbuffer_free(struct Ringbuffer* rb) { return sizeof(rb->buf) - (rb->head - rb->tail) - 1; } // size-1 .. 0
inline int      ringbuffer_empty(struct Ringbuffer* rb) { return rb->head == rb->tail; }

// _full tests for free < 2 instead of == 0 so that there's room for the final '\n' or 0x7e
inline int ringbuffer_full(struct Ringbuffer* rb) { return ringbuffer_free(rb) < 2; }

// serial_init() initializes USART{1,2,3}.
//
// The bytes will transmitted be as 8 data bits, no Parity bit, 1 stop bit (8N1) at the specified baud rate.
//
// Before calling serial_init, make sure to set up the GPIO pins: TX to AF_PP/10MHz. RX to IN FLOATING or Pull-up.
// and to enable the USART in the RCC register:	RCC->APBxENR |= RCC_APBxENR_USARTyEN;
void serial_init(struct USART_Type* usart, int baud, struct Ringbuffer* txbuf);

// serial_hdlc_tx() copies bufp[0:size] to an internal ringbuffer and schedules it for transmission as a HDLC frame.
//
// The data are suffixed with the 2 bytes of the x25 crc16 (polynomial 0x1021, start 0xffff)
// (https://play.golang.org/p/Irj52IfcJPt). Data plus checksum is then scrambled with an additive LSFR of period 256
// with polynomial 0x170 (https://play.golang.org/p/zNxcH-Zs4Pj) to make the occurence of values that must be escaped
// close to 1 in 64 bytes.
//
// The resulting bytes are then HDLC escaped and terminated with a FLAG (0x7e)
//
// The function returns 0 on succes, -1 if the buffer is temporarily too full and -2 if size exeeds the size of the
// allocated ring buffer.
int serial_hdlc_tx(struct USART_Type* usart, const uint8_t* bufp, size_t size);

// serial_hdlc_abort() aborts any current transmission with an ABORT character and flushes the ringbuffer.
// if no transmission is in progress, the call has no effect.
void serial_hdlc_abort(struct USART_Type* usart);

// serial_printf() interprets fmt as a format string for the variable parameters and copies a corresponding
// message to the usarts ringbuffer for transmission.  if the buffer is full, zaps the buffer and prints "!OVFL!".
//
int serial_printf(struct USART_Type* usart, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

inline void serial_wait(struct USART_Type* usart) {
	while (usart->CR1 & USART_CR1_TXEIE)
		;
}