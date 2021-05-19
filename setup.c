/*
 * setup.c
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"

static void change_RCC_HSI(); // chyba nie trzeba nic dodawac
static void setup_USART2(); // USART for radioreceiver
static void setup_USART1(); // USART for sending data (bluetooth or radio)
static void setup_GPIOA(); // ------------- GPIOA (pin 0 - TIM2_CH1, pin 1 - TIM2_CH2, pin 2 - TX, pin 15 - RX) -------------
static void setup_GPIOB(); // ------------- GPIOB (pin 6 - TX, pin 7 - RX, pin 8 - SCL, pin 9 - SDA, pin 10 - TIM2_CH3, pin 11 - TIM2_CH4) -------------
static void setup_GPIOC(); // ------------- GPIOC (pin 9 - INT from IMU)
static void setup_TIM2(); // setup TIM2
static void setup_TIM21(); // setup TIM21
static void setup_TIM6(); //setup TIM6
static void setup_I2C1();
//static void setup_NVIC();
static void setup_PLL();
static void setup_EXTI();
static void setup_DMA();

extern uint8_t table_of_bytes_to_sent[2 * ALL_ELEMENTS_TO_SEND + 4];
extern uint8_t read_write_tab[];
extern volatile uint8_t rxBuf[];
void setup() {
	change_RCC_HSI();
	setup_PLL();
	setup_USART1();
	setup_USART2();
	setup_GPIOA();
	setup_GPIOB();
	setup_GPIOC();
	setup_TIM2();
	setup_TIM21();
	setup_TIM6();
	setup_I2C1();
	setup_EXTI();
	setup_DMA();
	//setup_NVIC();

}

static void change_RCC_HSI() {
	// enable HSI:
	RCC->CR |= RCC_CR_HSION;
	while (0 == (RCC->CR & RCC_CR_HSIRDY)) {
		// waiting until RDY bit is set
	}
	// set SYSCLK to HSI16:
	RCC->CFGR |= RCC_CFGR_SW_0;
}

static void setup_PLL() {

	/* (1) Test if PLL is used as System clock */
	/* (2) Select HSI as system clock */
	/* (3) Wait for HSI switched */
	/* (4) Disable the PLL */
	/* (5) Wait until PLLRDY is cleared */
	/* (6) Set latency to 1 wait state */
	/* (7) Set the PLL multiplier to 24 and divider by 3 */
	/* (8) Enable the PLL */
	/* (9) Wait until PLLRDY is set */
	/* (10) Select PLL as system clock */
	/* (11) Wait until the PLL is switched on */
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
	{
		RCC->CFGR = (RCC->CFGR & (uint32_t) (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}
	RCC->CR &= (uint32_t) (~RCC_CR_PLLON);/* (4) */
	while ((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
	{
		/* For robust implementation, add here time-out management */
	}
	FLASH->ACR |= FLASH_ACR_LATENCY; /* (6) */
	RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV)))
			| (RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2); /* (7) */
	RCC->CR |= RCC_CR_PLLON; /* (8) */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) /* (9) */
	{
		/* For robust implementation, add here time-out management */
	}
	RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* (10) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (11) */
	{
		/* For robust implementation, add here time-out management */
	}
}

static void setup_USART2() {
	// enable USART2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR = 278 - 1; // 32 000 000 / 115 200 = 277.78
	USART2->CR3 |= USART_CR3_OVRDIS;
	//USART2->CR3 |= USART_CR3_DMAR;    //uncomment for DMA reading
	USART2->CR1 |=
	USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE
			| USART_CR1_IDLEIE;
}

static void setup_USART1() {
	// enable USART1 clock:
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 278 - 1; // 32 000 000 / 115 200 = 277.78
	USART1->CR3 |= USART_CR3_OVRDIS;
	USART1->CR3 |= USART_CR3_DMAT;
	USART1->CR1 |=
	USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

}

static void setup_GPIOA() {
	// enable GPIOA clock:
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	//	set mode (alternate function):
	GPIOA->MODER &= ~( GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0);
	GPIOA->MODER |= ( GPIO_MODER_MODE0_1);

	GPIOA->MODER &= ~( GPIO_MODER_MODE1_1 | GPIO_MODER_MODE1_0);
	GPIOA->MODER |= ( GPIO_MODER_MODE1_1);

	GPIOA->MODER &= ~( GPIO_MODER_MODE2_1 | GPIO_MODER_MODE2_0);
	GPIOA->MODER |= ( GPIO_MODER_MODE2_1);

	GPIOA->MODER &= ~(GPIO_MODER_MODE15_1 | GPIO_MODER_MODE15_0);
	GPIOA->MODER |= (GPIO_MODER_MODE15_1);

	//	set alternate functions:
	GPIOA->AFR[0] &= ~0x00000FFF;
	GPIOA->AFR[0] |= 0x00000422;
	GPIOA->AFR[1] &= ~0xF0000000;
	GPIOA->AFR[1] |= 0x40000000;

	GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEED2_1 | GPIO_OSPEEDER_OSPEED2_0
			| GPIO_OSPEEDER_OSPEED1_1 | GPIO_OSPEEDER_OSPEED1_0
			| GPIO_OSPEEDER_OSPEED15_1 | GPIO_OSPEEDER_OSPEED15_0);

}

static void setup_GPIOB() {
	// enable GPIOB clock:
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

	//	set mode (alternate function 10):
	GPIOB->MODER &= ~( GPIO_MODER_MODE6_1 | GPIO_MODER_MODE6_0);
	GPIOB->MODER |= ( GPIO_MODER_MODE6_1);

	GPIOB->MODER &= ~( GPIO_MODER_MODE7_1 | GPIO_MODER_MODE7_0);
	GPIOB->MODER |= ( GPIO_MODER_MODE7_1);

	GPIOB->MODER &= ~( GPIO_MODER_MODE8_1 | GPIO_MODER_MODE8_0);
	GPIOB->MODER |= ( GPIO_MODER_MODE8_1);

	GPIOB->MODER &= ~(GPIO_MODER_MODE9_1 | GPIO_MODER_MODE9_0);
	GPIOB->MODER |= (GPIO_MODER_MODE9_1);

	GPIOB->MODER &= ~(GPIO_MODER_MODE10_1 | GPIO_MODER_MODE10_0);
	GPIOB->MODER |= (GPIO_MODER_MODE10_1);

	GPIOB->MODER &= ~(GPIO_MODER_MODE11_1 | GPIO_MODER_MODE11_0);
	GPIOB->MODER |= (GPIO_MODER_MODE11_1);

	//	set alternate functions:
	GPIOB->AFR[1] &= ~0x0000FFFF;
	GPIOB->AFR[1] |= 0x00002244;

	// Output open-drain (pin 8,9):
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);

	// No pull-up No pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD8_1
			| GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD9_1);

	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEED6_1 | GPIO_OSPEEDER_OSPEED6_0
			| GPIO_OSPEEDER_OSPEED7_1 | GPIO_OSPEEDER_OSPEED7_0
			| GPIO_OSPEEDER_OSPEED8_1 | GPIO_OSPEEDER_OSPEED8_0
			| GPIO_OSPEEDER_OSPEED9_1 | GPIO_OSPEEDER_OSPEED9_0
			| GPIO_OSPEEDER_OSPEED10_1 | GPIO_OSPEEDER_OSPEED10_0
			| GPIO_OSPEEDER_OSPEED11_1 | GPIO_OSPEEDER_OSPEED11_0);

}

static void setup_GPIOC() {
	// enable GPIOC clock:
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;

	//	set mode (input 00):
	GPIOC->MODER &= ~( GPIO_MODER_MODE9_1 | GPIO_MODER_MODE9_0
			| GPIO_MODER_MODE8_1 | GPIO_MODER_MODE8_0 | GPIO_MODER_MODE3_1
			| GPIO_MODER_MODE3_0);

	//pull-down (10):
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD9_0 | GPIO_PUPDR_PUPD9_1
			| GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD3_0
			| GPIO_PUPDR_PUPD3_1);
	GPIOC->PUPDR |=
	GPIO_PUPDR_PUPD9_1 | GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD3_1;

	GPIOC->OSPEEDR |= ( GPIO_OSPEEDER_OSPEED9_1 | GPIO_OSPEEDER_OSPEED9_0
			| GPIO_OSPEEDER_OSPEED8_1 | GPIO_OSPEEDER_OSPEED8_0
			| GPIO_OSPEEDER_OSPEED3_1 | GPIO_OSPEEDER_OSPEED3_0);
}

static void setup_TIM2() {
// enable TIM2 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
// register is buffered:
	TIM2->CR1 |= TIM_CR1_ARPE;
// PWM mode 1 and output compare 1 preload enable:
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
// PWM mode 1 and output compare 2 preload enable:
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
// PWM mode 1 and output compare 3 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
// PWM mode 1 and output compare 4 preload enable:
	TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

//channel 1 enable:
	TIM2->CCER |= TIM_CCER_CC1E;
//channel 2 enable:
	TIM2->CCER |= TIM_CCER_CC2E;
//channel 3 enable:
	TIM2->CCER |= TIM_CCER_CC3E;
//channel 4 enable:
	TIM2->CCER |= TIM_CCER_CC4E;

	TIM2->PSC = 32 - 1; 		// zeby counter liczyl mikrosekundy
	TIM2->ARR = 20000 - 1; 		// 1 okres pwm trwa 20[ms]

	TIM2->CCR1 = 1000 - 1; 			//wypelneinie channel 1
	TIM2->CCR2 = 1000 - 1; 			//wypelneinie channel 2
	TIM2->CCR3 = 1000 - 1; 			//wypelneinie channel 3
	TIM2->CCR4 = 1000 - 1; 			//wypelneinie channel 4

	//	TIM2 enabling:
	TIM2->CR1 |= TIM_CR1_CEN;
}

static void setup_TIM21() {
// enable TIM21 clock:
	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
// register is buffered:
	TIM21->CR1 |= TIM_CR1_ARPE;

	TIM21->PSC = (32 - 1) * 10; 			// every 10 us 1 count
	TIM21->ARR = 65536 - 1; 		// 1 period is 0.65536 s long

	//	TIM21 enabling:
	TIM21->CR1 |= TIM_CR1_CEN;
}
static void setup_TIM6() {
// enable TIM6 clock:
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
// register is buffered:
	TIM6->CR1 |= TIM_CR1_ARPE;

	TIM6->PSC = 32 - 1; 			// every 1 us 1 count
	TIM6->ARR = 65536 - 1; 		// 1 period is 0.065536 s long

	TIM6->DIER |= TIM_DIER_UIE;
	//	TIM6 enabling:
	TIM6->CR1 |= TIM_CR1_CEN;
}

static void setup_I2C1() {
	//	enable I2C clock:
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	//	I2C1 clock source selection (HSI16):
	RCC->CCIPR |= RCC_CCIPR_I2C1SEL_1;
	//	400Hz times setting:
	I2C1->TIMINGR = (uint32_t) 0x10320309;
	//DMA receiving enable:
	I2C1->CR1 |= I2C_CR1_RXDMAEN;
	//	peripheral enable:
	I2C1->CR1 |= I2C_CR1_PE;
	// autoend enable
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	// interrupt from I2C1 not masked:
	EXTI->IMR |= EXTI_IMR_IM23;
}
static void setup_EXTI() {

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//set PC9 as EXTI
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PC;
	//disable mask for IM9 and IM15
	EXTI->IMR &= ~(EXTI_IMR_IM9 | EXTI_IMR_IM15);
	EXTI->IMR |= EXTI_IMR_IM9 | EXTI_IMR_IM15;
	//setting rising edge detection:
	EXTI->RTSR |= EXTI_RTSR_RT9;
}

static void setup_DMA() {
	//USART1 telemetry transmiting:
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_CSELR->CSELR |= 0b0011 << 4;
	DMA1_Channel2->CPAR = (uint32_t) (&(USART1->TDR));
	DMA1_Channel2->CMAR = (uint32_t) (table_of_bytes_to_sent);
	DMA1_Channel2->CNDTR = 2 * ALL_ELEMENTS_TO_SEND + 4;
	DMA1_Channel2->CCR &= 0x000;
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR
			| DMA_CCR_TCIE;
	//USART2 RX reading:
	DMA1_CSELR->CSELR |= 0b0100 << 16;
	DMA1_Channel5->CPAR = (uint32_t) (&(USART2->RDR));
	DMA1_Channel5->CMAR = (uint32_t) (&rxBuf[1]);
	DMA1_Channel5->CNDTR = 31;
	DMA1_Channel5->CCR &= 0x000;
	DMA1_Channel5->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE
			| DMA_CCR_PL_0;

	//IMU reading:
	DMA1_CSELR->CSELR |= 0b0110 << 8;
	DMA1_Channel3->CPAR = (uint32_t) (&(I2C1->RXDR));
	DMA1_Channel3->CMAR = (uint32_t) (read_write_tab);
	DMA1_Channel3->CNDTR = 14;
	DMA1_Channel3->CCR &= 0x000;
	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE
			| DMA_CCR_PL_1;

}
void setup_NVIC() {
	// nvic interrupt enable (USART2 interrupt):
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 4);
	// nvic interrupt enable (USART1 interrupt):
	NVIC_EnableIRQ(USART1_IRQn);
	// I2C1 interrupt enable:
//	NVIC_EnableIRQ(I2C1_IRQn);
	// nvic interrupt enable (TIM6 interrupt);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	// EXTI interrupt enable:
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	NVIC_SetPriority(EXTI4_15_IRQn, 3);

	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2);

	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 1);

}
