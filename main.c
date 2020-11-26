/*
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
#include "serial.h"

/*
    STM32F013CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   -----------    ---------------------------------------
    PA0
    PA1
    PA2
    PA3
    PA4
    PA5
    PA6
    PA7
    PA8
    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp
    PA11
    PA12
    PA13  JTMS/SWDIO
    PA14  JTCK/SWCLK
    PA15

    PB0
    PB1
    PB2
    PB3
    PB4
    PB5
    PB6
    PB7
    PB8
    PB9
    PB10
    PB11
    PB12
    PB13
    PB14
    PB15

    PC13  LED0         out   OUT_OD 2MHz
    PC14
    PC15

*/

struct USART_Type* const USART_CONS = &USART1;

enum {
	USART1_TX_PIN = PA9,
	USART1_RX_PIN = PA10,
	LED0_PIN      = PC13,
};

static struct gpio_config_t {
	enum GPIO_Pin  pins;
	enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, {PBAll, Mode_IN}, {USART1_TX_PIN, Mode_AF_PP_50MHz}, {USART1_RX_PIN, Mode_IPU}, {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

static struct Ringbuffer usart1tx;

// periodic prompt to read accel temp
void TIM4_IRQ_Handler(void) {
	uint64_t now = cycleCount();
	if ((TIM4.SR & TIM_SR_UIF) == 0)
		return;
	TIM4.SR &= ~TIM_SR_UIF;

	(void)now;
}

/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
	enum IRQn_Type irq;
	uint8_t        group, sub;
} irqprios[] = {
 	{SysTick_IRQn,	0, 0},  // just increment timecounter, needed for high precision
 	{USART1_IRQn,	3, 1},	// i/o
 	{TIM4_IRQn,		3, 3},  // ~1Hz 
 	{None_IRQn,	0xff, 0xff},
};
/* clang-format on */

int main(void) {

	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	SysTick_Config(1U << 24); // tick at 72Mhz/2^24 = 4.2915 HZ

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
	for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
		NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
	}

	RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
	// RCC.APB1ENR |=;
	// RCC.AHBENR |= RCC_AHBENR_DMA1EN;

	delay(10);

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	led0_on();
	gpioLock(PAAll);
	gpioLock(PBAll);
	gpioLock(PCAll);

	serial_init(USART_CONS, 8 * 115200, &usart1tx);

	serial_printf(USART_CONS, "SWREV:%s\n", __REVISION__);
	serial_printf(USART_CONS, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
	              rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
	serial_printf(USART_CONS, "CPUID:%08lx\n", SCB.CPUID);
	serial_printf(USART_CONS, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	serial_wait(USART_CONS);

	// enable ~1Hz TIM4 to generate ...
	TIM4.DIER |= TIM_DIER_UIE;
	TIM4.PSC = 7200 - 1;  // 72MHz / 7200 = 10Khz
	TIM4.ARR = 11003 - 1; // 10KHz/11000 = .909Hz, avoid resonance by choosing a nice prime number
	TIM4.CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM4_IRQn);

	// Initialize the independent watchdog
	while (IWDG.SR != 0)
		__NOP();

	IWDG.KR  = 0x5555; // enable watchdog config
	IWDG.PR  = 0;      // prescaler /4 -> 10khz
	IWDG.RLR = 3200;   // count to 3200 -> 320ms timeout
	IWDG.KR  = 0xcccc; // start watchdog countdown

	uint64_t now       = cycleCount();
	uint64_t laststats = now;
	uint64_t dutyc     = 0;
	uint32_t mainloops = 0;
	uint32_t idleloops = 0;

	led0_off();

	for (;; ++mainloops) {

		__WFI(); // wait for interrupt to change the state of any of the subsystems
		now = cycleCount();

		// do stuff


		if (laststats + CLOCKSPEED_HZ < now) {
			// serial_printf(USART_CONS,"\033[H");
			serial_printf(USART_CONS, "cycles %li idle %li duty %lli / %lli us\n", mainloops, idleloops, dutyc / C_US,
			              (now - laststats) / C_US);
			serial_wait(USART_CONS);
			laststats = now;
			dutyc     = 0;
			mainloops = 0;
			idleloops = 0;
		}

		if (1) {
			IWDG.KR = 0xAAAA;
		}

		uint64_t active = cycleCount() - now;
		dutyc += active;

	} // forever

	return 0;
}
