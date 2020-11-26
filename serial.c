
#include "serial.h"

extern inline uint16_t ringbuffer_avail(struct Ringbuffer* rb);
extern inline uint16_t ringbuffer_free(struct Ringbuffer* rb);
extern inline int      ringbuffer_empty(struct Ringbuffer* rb);
extern inline int      ringbuffer_full(struct Ringbuffer* rb);

static inline void put_head(struct Ringbuffer* rb, uint8_t c) { rb->buf[rb->head++ % sizeof(rb->buf)] = c; }

extern inline void serial_wait(struct USART_Type* usart);

static struct Ringbuffer* tx_buf[3]      = {NULL, NULL, NULL};
static volatile uint16_t  tx_hdlcmode[3] = {0, 0, 0}; // 0x8000 | last char

static inline int usart_index(struct USART_Type* usart) {
	if (usart == &USART1)
		return 0;
	if (usart == &USART2)
		return 1;
	if (usart == &USART3)
		return 2;

	for (;;)
		__NOP(); // hang

	return 0;
}

enum {
	HDLC_ESC   = 0x7d, // 0b0111 1101
	HDLC_FLAG  = 0x7e, // 0b0111 1110
	HDLC_ABORT = 0x7f, // 0b0111 1111
};

void serial_init(struct USART_Type* usart, int baud, struct Ringbuffer* txbuf) {
	const int idx = usart_index(usart);

	tx_buf[idx]      = txbuf;
	tx_hdlcmode[idx] = 0;

	usart->CR1 = 0;
	usart->CR2 = 0;
	usart->CR3 = 0;

	// PCLK2 for USART1, PCLK1 for USART 2, 3
	// this depends on SetSystemclockTo72MHz setting pclk1 to sysclk/2 and pclk1 to sysclc/2
	uint32_t clk = 72000000;
	if (usart != &USART1) {
		clk /= 2;
	}
	usart->BRR = clk / baud;

	static const enum IRQn_Type irqn[3] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
	NVIC_EnableIRQ(irqn[idx]);

	usart->CR1 |= USART_CR1_UE | USART_CR1_TE;
}

static inline void irqHandler(struct USART_Type* usart, struct Ringbuffer* rb, volatile uint16_t* hdlcmode) {
	if (*hdlcmode && ringbuffer_empty(rb)) {
		// last sent was FLAG or ABORT
		if ((*hdlcmode & HDLC_FLAG) == HDLC_FLAG) {
			put_head(rb, *hdlcmode);
		}
	}

	if (!ringbuffer_empty(rb)) {
		if ((usart->SR & USART_SR_TXE) != 0) {
			uint8_t b = rb->buf[rb->tail++ % sizeof(rb->buf)];
			usart->DR = b;
			*hdlcmode = (*hdlcmode & 0xff00) | b;
		}
	} else {
		usart->CR1 &= ~USART_CR1_TXEIE;
	}
	return;
}

// IRQ Handlers end up in the IRQ vector table via startup_*.s
void USART1_IRQ_Handler(void) { irqHandler(&USART1, tx_buf[0], &tx_hdlcmode[0]); }
void USART2_IRQ_Handler(void) { irqHandler(&USART2, tx_buf[1], &tx_hdlcmode[1]); }
void USART3_IRQ_Handler(void) { irqHandler(&USART3, tx_buf[2], &tx_hdlcmode[2]); }

static inline void put_head_esc(struct Ringbuffer* rb, uint8_t c) {
	switch (c) {
	case HDLC_ESC:
	case HDLC_FLAG:
	case HDLC_ABORT:
		c ^= 0x20;
		put_head(rb, HDLC_ESC);
	}
	put_head(rb, c);
}

int serial_hdlc_tx(struct USART_Type* usart, const uint8_t* bufp, size_t size) {
	const int          idx = usart_index(usart);
	struct Ringbuffer* tx  = tx_buf[idx];
	tx_hdlcmode[idx]       = 0x8000;
	usart->CR1 |= USART_CR1_TXEIE; // enable transmissions

	uint16_t       x25crc = 0xffff;
	uint8_t        lfsr   = 0x1;
	const uint8_t* p      = bufp;

	while ((size > 0) && !ringbuffer_full(tx)) {
		uint8_t t = x25crc;
		t ^= *p;
		t ^= t << 4;
		uint16_t u = t;
		x25crc     = (x25crc >> 8) ^ (u << 8) ^ (u << 3) ^ (u >> 4);

		if (lfsr & 1) {
			lfsr >>= 1;
			lfsr ^= 0xB8;
		} else {
			lfsr >>= 1;
		}

		put_head_esc(tx, *p ^ lfsr);

		++p;
		--size;
	}

	if (ringbuffer_free(tx) < 5) {
		put_head(tx, HDLC_ABORT);
		// in case the irq handler caught up and switched off
		usart->CR1 |= USART_CR1_TXEIE; // enable transmissions
		return -1;
	}

	put_head_esc(tx, x25crc >> 8);
	put_head_esc(tx, x25crc);
	put_head(tx, HDLC_FLAG);
	// in case the irq handler caught up and switched off
	usart->CR1 |= USART_CR1_TXEIE; // enable transmissions
	return 0;
}

#define STB_SPRINTF_STATIC
#define STB_SPRINTF_MIN 32
#define STB_SPRINTF_NOFLOAT
#define STB_SPRINTF_IMPLEMENTATION

#include "stb_sprintf.h"

static char* rb_putcb(char* buf, void* user, int len) {
	struct Ringbuffer* rb = (struct Ringbuffer*)user;
	if (ringbuffer_free(rb) < len) {
		rb->head = rb->tail;
		put_head(rb, '!');
		put_head(rb, 'O');
		put_head(rb, 'V');
		put_head(rb, 'F');
		put_head(rb, '!');
	}
	for (int i = 0; i < len; ++i) {
		put_head(rb, buf[i]);
	}
	return buf;
}

int serial_printf(struct USART_Type* usart, const char* fmt, ...) {
	const int          idx = usart_index(usart);
	struct Ringbuffer* tx  = tx_buf[idx];
	tx_hdlcmode[idx]       = 0;
	stbsp_set_separators('\'', '.');

	va_list ap;
	va_start(ap, fmt);
	char b[STB_SPRINTF_MIN];
	int  rv = stbsp_vsprintfcb(rb_putcb, tx, b, fmt, ap);
	va_end(ap);
	usart->CR1 |= USART_CR1_TXEIE; // enable transmissions

	return rv;
}
