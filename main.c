/*
 *  At 100Hz ADC sample channels 0..10 and print to UART TX1 (921600 8N1)
 *  Output is prefixed with time in us and count
 * 
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
#include "serial.h"

/*
    STM32F013CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   -----------    ---------------------------------------
    PA0:7 ADC 0:7      in    Analog         Analog input 0-7

    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp

    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer

    PB0:1 ADC 8:9      in    Analog         Analog input 8,9

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED

*/

enum {
	ADC0_7_PIN    = PA0|PA1|PA2|PA3|PA4|PA5|PA6|PA7,
	ADC8_9_PIN    = PB0|PB1,
	USART1_TX_PIN = PA9,
	USART1_RX_PIN = PA10,
	LED0_PIN      = PC13,
};

static struct gpio_config_t {
	enum GPIO_Pin  pins;
	enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, // reset
    {PBAll, Mode_IN}, // reset
    {PCAll, Mode_IN}, // reset
    {ADC0_7_PIN, Mode_INA},
    {ADC8_9_PIN, Mode_INA},
    {USART1_TX_PIN, Mode_AF_PP_50MHz}, 
    {USART1_RX_PIN, Mode_IPU}, 
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

static struct Ringbuffer usart1tx;

static volatile uint64_t adctrig  = 0; // time last ADC trigger
static volatile uint64_t adcdone  = 0; // time of last DMA transfer completion
static volatile uint64_t adccount = 0;
static uint32_t adcdata[16]; // up to 16 samples

// ADC1 DMA Transfer Complete 
void DMA1_Channel1_IRQ_Handler(void) {
    adcdone = cycleCount();
    ++adccount;
    DMA1.IFCR = DMA1.ISR & 0x000f; // clear all dma interrupt flags TODO only xferdone so others cause hang
}

// Trigger of ADC1 scan
void TIM3_IRQ_Handler(void) {
    if ((TIM3.SR & TIM_SR_UIF) == 0)
            return;
    adctrig = cycleCount();
    TIM3.SR &= ~TIM_SR_UIF;
 }


/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
	enum IRQn_Type irq;
	uint8_t        group, sub;
} irqprios[] = {
 	{SysTick_IRQn,       0, 0},
 	{DMA1_Channel1_IRQn, 1, 0},
 	{TIM3_IRQn,          2, 0},
 	{USART1_IRQn,        3, 0},
 	{None_IRQn,	0xff, 0xff},
};
/* clang-format on */

void main(void) {

	uint8_t rf = (RCC.CSR >> 24) & 0xfc;
	RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

	SysTick_Config(1U << 24); // tick at 72Mhz/2^24 = 4.2915 HZ

	NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
	for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
		NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
	}

	RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
	RCC.APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC.AHBENR  |= RCC_AHBENR_DMA1EN;
 	RCC.CFGR    |= RCC_CFGR_ADCPRE_DIV6; // ADC clock 72/6 MHz = 12 Mhz, must be < 14MHz

	delay(10); // let all clocks and peripherals start up

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);
	gpioLock(PCAll);

	led0_off();

	serial_init(&USART1, 921600, &usart1tx);

	serial_printf(&USART1, "SWREV:%s\n", __REVISION__);
	serial_printf(&USART1, "CPUID:%08lx\n", SCB.CPUID);
	serial_printf(&USART1, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	serial_printf(&USART1, "RESET:%02x%s%s%s%s%s%s\n", rf, 
			rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "", rf & 0x10 ? " SFT" : "", 
			rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
	serial_wait(&USART1);


 	ADC1.CR2 |= ADC_CR2_ADON;       // first wake up
 	ADC2.CR2 |= ADC_CR2_ADON;
 	delay(10);
 	ADC1.CR2 |= ADC_CR2_CAL;        // start calibrate
 	ADC2.CR2 |= ADC_CR2_CAL;
 	while (ADC1.CR2 & ADC_CR2_CAL)  // wait until done
 		__NOP();
 	while (ADC2.CR2 & ADC_CR2_CAL)
 		__NOP();

 	// scan adc0..9 triggered by timer 3, using dma.
 	// use 13.5 cycles sample time +12.5 = 26 cycles at 12Mhz = 2.16667us per sample 
 	// so 21.6667 us per cycle, triggered every 10ms
 	ADC1.CR1 |= ADC_CR1_SCAN | (0b0110 << 16); 			 // simultaneous regular dual mode
 	ADC1.CR2 |= ADC_CR2_EXTTRIG | (4<<17) | ADC_CR2_DMA; // Trigger from Timer 3 TRGO event.

 	ADC2.CR1 |= ADC_CR1_SCAN; 			 
 	ADC2.CR2 |= ADC_CR2_EXTTRIG | (7<<17) | ADC_CR2_DMA; // Trigger must be set to sw.

 	ADC1.SMPR2 = 0b010010010010010010010010010010; 		 // SMPR0..9 to 010 ->  13.5 cycles
 	ADC2.SMPR2 = ADC1.SMPR2;                             // must be set to same

 	ADC1.SQR1 = (5-1) << 20; 							 // 5 conversions
 	ADC1.SQR3 = 0 | (2<<5) | (4<<10) | (6<<15) | (8<20); // channels 0 2 4 6 8 in that order

 	ADC2.SQR1 = (5-1) << 20; 							 // 5 conversions
 	ADC1.SQR3 = 1 | (3<<5) | (5<<10) | (7<<15) | (9<20); // channels 1 3 5 7 9 in that order


 	ADC1.CR2 |= ADC_CR2_ADON;         // up & go.
 	ADC2.CR2 |= ADC_CR2_ADON;         // up & go.


 	DMA1_Channel1.CCR   = (2<<10) | (2<<8) | DMA_CCR1_MINC | DMA_CCR2_CIRC | DMA_CCR1_TCIE;
    DMA1_Channel1.CPAR  = (uint32_t)&ADC1.DR;
    DMA1_Channel1.CMAR  = (uint32_t)&adcdata[0];
    DMA1_Channel1.CNDTR = ((ADC1.SQR1 >> 20) & 0xf) + 1; // number of conversions in scan
	DMA1_Channel1.CCR  |= DMA_CCR1_EN;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// enable 100Hz TIM3 to trigger ADC
	TIM3.DIER |= TIM_DIER_UIE;
	TIM3.PSC = 7200 - 1;  	// 72MHz / 7200 = 10Khz
	TIM3.ARR = 100 - 1; 	// 10KHz/100 = 100Hz
	TIM3.CR2 |= (2<<4);     // TRGO is update event
	TIM3.CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn);

	// ADC/DMA errors will cause the watchdog to cease being triggered
	// Initialize the independent watchdog
	while (IWDG.SR != 0)
		__NOP();

	IWDG.KR  = 0x5555; // enable watchdog config
	IWDG.PR  = 0;      // prescaler /4 -> 10kHz
	IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
	IWDG.KR  = 0xcccc; // start watchdog countdown

	uint64_t lastreport = 0;
	int len = ((ADC1.SQR1 >> 20) & 0xf) + 1;  // number of conversions

	for (;;) {
		__WFI(); // wait for interrupt to change the state of any of the subsystems
		if (lastreport == adccount)
			continue;

		int64_t skip = adccount - lastreport;
		lastreport = adccount;

		if (skip > 1) {
			serial_printf(&USART1, "### skipped %lld\n", skip);
			led0_on();
		}
	
#if 1
		uint64_t us = adctrig/C_US;
		uint64_t s = us / 1000000;
		us = us % 1000000;
		serial_printf(&USART1, "%lli.%06lli %lli %lli", s, us, adccount, (adcdone-adctrig)/C_US);
#else
		serial_printf(&USART1, "%lli", adccount);
#endif

		for (int i = 0; i < len; i++) {
			serial_printf(&USART1, " %4d %4d", (int)(adcdata[i]&0xfff), (int)(adcdata[i]>>16));
		}
		serial_printf(&USART1, "\n");

		IWDG.KR = 0xAAAA; 		// kick the watchdog

		if (lastreport != adccount) {
			serial_printf(&USART1, "### overflow\n");
			led0_on();
		}

	} // forever
}
