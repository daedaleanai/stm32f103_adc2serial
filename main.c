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
    PA8
    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp
    PA11
    PA12
    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer
    PA15

    PB0:1 ADC 8:9      in    Analog         Analog input 8,9
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

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED
    PC14
    PC15

*/

struct USART_Type* const USART_CONS = &USART1;

enum {
	USART1_TX_PIN = PA9,
	USART1_RX_PIN = PA10,
	LED0_PIN      = PC13,
	ADC0_7_PIN    = PA0|PA1|PA2|PA3|PA4|PA5|PA6|PA7,
	ADC8_9_PIN    = PB0|PB1,
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

static volatile uint64_t adccount = 0;
static volatile uint64_t adctime  = 0; // time of last DMA transfer completion
static uint32_t adcdata[16]; // up to 16 samples

// ADC1 DMA Transfer Complete 
void DMA1_Channel1_IRQ_Handler(void) {
        DMA1.IFCR = DMA1.ISR & 0x000f;
        adctime = cycleCount();
        ++adccount;
}



/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
	enum IRQn_Type irq;
	uint8_t        group, sub;
} irqprios[] = {
 	{SysTick_IRQn,       0, 0},
 	{DMA1_Channel1_IRQn, 1, 0},
 	{USART1_IRQn,        3, 0},
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

	RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_ADC1EN;
	// RCC.APB1ENR |=;
	// RCC.AHBENR |= RCC_AHBENR_DMA1EN;
 	RCC.CFGR |= RCC_CFGR_ADCPRE_DIV6; // ADC clock 72/6 MHz = 12 Mhz, must be < 14MHz


	delay(10);

	for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
		gpioConfig(p->pins, p->mode);
	}

	gpioLock(PAAll);
	gpioLock(PBAll);
	gpioLock(PCAll);

	serial_init(USART_CONS, 921600, &usart1tx);

	serial_printf(USART_CONS, "SWREV:%s\n", __REVISION__);
	serial_printf(USART_CONS, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "",
	              rf & 0x20 ? " IWDG" : "", rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
	serial_printf(USART_CONS, "CPUID:%08lx\n", SCB.CPUID);
	serial_printf(USART_CONS, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
	serial_wait(USART_CONS);


 	ADC1.CR2 |= ADC_CR2_ADON;       // first wake up
 	delay(10);
 	ADC1.CR2 |= ADC_CR2_CAL;        // start calibrate
 	while (ADC1.CR2 & ADC_CR2_CAL)  // wait until done
 		__NOP();

 	// scan adc0..9 triggered by timer 3, using dma.
 	// use 13.5 cycles sample time +12.5 = 26 cycles at 12Mhz = 2.16667us per sample so 21.6667 us per cycle, triggered every 10ms
 	ADC1.CR1 |= ADC_CR1_SCAN ;
 	ADC1.CR2 |= ADC_CR2_EXTTRIG | (4<<17) | ADC_CR2_DMA; // Trigger from Timer 3 TRGO event.
 	ADC1.SMPR2 = 0b010010010010010010010010010010; 		 // SMPR0..9 to 010 ->  13.5 cycles
 	ADC1.SQR1 = (10-1) << 20; 							 // 10 conversions
 	ADC1.SQR2 = 6 | (7<<5) | (8<<10) | (9<<15);   		 // channels 0...9 in that order
 	ADC1.SQR3 = 0 | (1<<5) | (2<<10) | (3<<15) | (4<<20) | (5<<25);
 	ADC1.CR2 |= ADC_CR2_ADON;         // up & go.

 	DMA1_Channel1.CCR   = (2<<10) | (2<<8) | DMA_CCR1_MINC | DMA_CCR2_CIRC | DMA_CCR1_TCIE;
    DMA1_Channel1.CPAR  = (uint32_t)&ADC1.DR;
    DMA1_Channel1.CMAR  = (uint32_t)&adcdata[0];
    DMA1_Channel1.CNDTR = ((ADC1.SQR1 >> 20) & 0xf) + 1; // number of conversions in scan
	DMA1_Channel1.CCR  |= DMA_CCR1_EN;
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// enable 100Hz TIM3 to trigger ADC
	TIM3.PSC = 7200 - 1;  	// 72MHz / 7200 = 10Khz
	TIM3.ARR = 100 - 1; 	// 10KHz/100 = 100Hz
	TIM3.CR2 |= (2<<4);     // TRGO is update event
	TIM3.CR1 |= TIM_CR1_CEN;

	// ADC/DMA errors will cause the watchdog to cease being triggered
	// Initialize the independent watchdog
	while (IWDG.SR != 0)
		__NOP();

	IWDG.KR  = 0x5555; // enable watchdog config
	IWDG.PR  = 0;      // prescaler /4 -> 10kHz
	IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
	IWDG.KR  = 0xcccc; // start watchdog countdown

	uint64_t lastreport = 0;
	int len = ((ADC1.SQR1 >> 20) & 0xf) + 1;

	for (;;) {
		__WFI(); // wait for interrupt to change the state of any of the subsystems
		if (lastreport == adccount)
			continue;

		int64_t skip = adccount - lastreport;
		lastreport = adccount;

		if (skip > 1) {
			serial_printf(USART_CONS, "### skipped %lld\n", skip);
		}
	
		serial_printf(USART_CONS, "%lli %lli", adctime/C_US, adccount);

		for (int i = 0; i < len; i++) {
			serial_printf(USART_CONS, " %4d", (int)(adcdata[i]&0xfff));
		}
		serial_printf(USART_CONS, "\n");

		IWDG.KR = 0xAAAA;

		if (lastreport != adccount) {
			serial_printf(USART_CONS, "### overflow\n");
		}

	} // forever

	return 0;
}
