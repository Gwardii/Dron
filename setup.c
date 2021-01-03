/*
 * setup.c
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "setup.h"

static void change_RCC_HSI(); // chyba nie trzeba nic dodawac
static void setup_USART2(); // USART for radioreceiver
static void setup_GPIOA();// ------------- GPIOA (pin 0 - TIM2_CH1, pin 1 - TIM2_CH2, pin 2 - TX, pin 15 - RX) -------------
static void setup_GPIOB();// ------------- GPIOB (pin 8 - SCL, pin 9 - SDA, pin 10 - TIM2_CH3, pin 11 - TIM2_CH4) -------------
static void setup_TIM2();// setup TIM2
static void setup_I2C1();
static void setup_NVIC();

void setup(){
	change_RCC_HSI();
	setup_USART2();
	setup_GPIOA();
	setup_GPIOB();
	setup_TIM2();
	setup_I2C1();
	setup_NVIC();
}

static void change_RCC_HSI(){
	// enable HSI:
		RCC->CR |= RCC_CR_HSION;
		while (0 == (RCC->CR  & RCC_CR_HSIRDY)) {
		// waiting until RDY bit is set
		}
		// set SYSCLK to HSI16:
		RCC->CFGR |= RCC_CFGR_SW_0;
}

static void setup_USART2(){
	// enable USART2 clock:
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		USART2->BRR = 139 - 1; // 16 000 000 / 115 200 = 138.88
		USART2->CR3 = USART_CR3_OVRDIS;
		USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
		USART2->ISR=USART_ISR_ORE;
}

static void setup_GPIOA(){
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
		GPIOA->MODER |= (GPIO_MODER_MODE15_1 );

		//	set alternate functions:
		GPIOA->AFR[0] &= ~0x00000FFF;
		GPIOA->AFR[0] |=  0x00000422;
		GPIOA->AFR[1] &= ~0xF0000000;
		GPIOA->AFR[1] |=  0x40000000;

		GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEED2_1 | GPIO_OSPEEDER_OSPEED2_0 | GPIO_OSPEEDER_OSPEED1_1 | GPIO_OSPEEDER_OSPEED1_0 | GPIO_OSPEEDER_OSPEED15_1 | GPIO_OSPEEDER_OSPEED15_0);
}

static void setup_GPIOB(){
	// enable GPIOB clock:
		RCC->IOPENR |=RCC_IOPENR_GPIOBEN;

		//	set mode (alternate function):
		GPIOB->MODER &= ~( GPIO_MODER_MODE8_1 | GPIO_MODER_MODE8_0);
		GPIOB->MODER |= ( GPIO_MODER_MODE8_1);

		GPIOB->MODER &= ~(GPIO_MODER_MODE9_1 | GPIO_MODER_MODE9_0);
		GPIOB->MODER |= (GPIO_MODER_MODE9_1 );

		GPIOB->MODER &= ~(GPIO_MODER_MODE10_1 | GPIO_MODER_MODE10_0);
		GPIOB->MODER |= (GPIO_MODER_MODE10_1 );

		GPIOB->MODER &= ~(GPIO_MODER_MODE11_1 | GPIO_MODER_MODE11_0);
		GPIOB->MODER |= (GPIO_MODER_MODE11_1 );

		//	set alternate functions:
		GPIOB->AFR[1] &= ~0x0000FFFF;
		GPIOB->AFR[1] |=  0x00002244;

		// Output open-drain (pin 8,9):
		GPIOB->OTYPER |=(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9);

		// No pull-up No pull-down
		GPIOB->PUPDR &=~(GPIO_PUPDR_PUPD8_0|GPIO_PUPDR_PUPD8_1|GPIO_PUPDR_PUPD9_0|GPIO_PUPDR_PUPD9_1);

		GPIOB->OSPEEDR |= ( GPIO_OSPEEDER_OSPEED8_1 | GPIO_OSPEEDER_OSPEED8_0 | GPIO_OSPEEDER_OSPEED9_1 | GPIO_OSPEEDER_OSPEED9_0 | GPIO_OSPEEDER_OSPEED10_1 | GPIO_OSPEEDER_OSPEED10_0 | GPIO_OSPEEDER_OSPEED11_1 | GPIO_OSPEEDER_OSPEED11_0);
}
static void setup_TIM2(){
	// enable TIM2 clock:
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		// register is buffered:
		TIM2->CR1 |=TIM_CR1_ARPE;
		// PWM mode 1 and output compare 1 preload enable:
		TIM2->CCMR1 |=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1PE;
		// PWM mode 1 and output compare 2 preload enable:
		TIM2->CCMR1 |=TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2PE;
		// PWM mode 1 and output compare 3 preload enable:
		TIM2->CCMR2 |=TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3PE;
		// PWM mode 1 and output compare 4 preload enable:
		TIM2->CCMR2 |=TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2|TIM_CCMR2_OC4PE;

		//channel 1 enable:
		TIM2->CCER |=TIM_CCER_CC1E;
		//channel 2 enable:
		TIM2->CCER |=TIM_CCER_CC2E;
		//channel 3 enable:
		TIM2->CCER |=TIM_CCER_CC3E;
		//channel 4 enable:
		TIM2->CCER |=TIM_CCER_CC4E;


		TIM2->PSC = 16-1; 			// zeby counter liczyl mikrosekundy
		TIM2->ARR = 20000 - 1; 		// 1 okres pwm trwa 20[ms]

		TIM2->CCR1 =1000 - 1; 			//wypelneinie channel 1
		TIM2->CCR2 =1000 - 1; 			//wypelneinie channel 2
		TIM2->CCR3 =1000 - 1; 			//wypelneinie channel 3
		TIM2->CCR4 =1000 - 1; 			//wypelneinie channel 4
}
static void setup_I2C1(){
	//	enable I2C clock:
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		//	I2C1 clock source selection (HSI16):
		RCC->CCIPR |=RCC_CCIPR_I2C1SEL_1;
		//	400Hz times setting:
		I2C1->TIMINGR = (uint32_t)0x10320309;
		//	peripheral enable:
		I2C1->CR1 = I2C_CR1_PE;
}
static void setup_NVIC(){
	// nvic interrupt enable (USART2 interrupt):
	NVIC_EnableIRQ(USART2_IRQn);
	// nvic interrupt enable (TIM2 interrupt):
	NVIC_EnableIRQ(TIM2_IRQn);

	//	TIM2 enabling:
	TIM2->CR1 |=TIM_CR1_CEN;

}
