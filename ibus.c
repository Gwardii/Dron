/*
 * ibus.c
 *
 *  Created on: 25.01.2021
 *      Author: filip
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "ibus.h"

volatile uint8_t rxBuf[32];
static uint8_t rxindex = 0;


static void failsafe_RX();

//for debugging only:
static int pok = 0;
static int pok1 = 0;

void DMA1_Channel4_5_6_7_IRQHandler(void) {
	//if channel4 transfer is completed:
	pok++;
	//if channel5 transfer is completed:
	if (DMA1->ISR & DMA_ISR_TCIF5) {
		DMA1->IFCR |= DMA_IFCR_CTCIF5;
		DMA1_Channel5->CCR &= ~DMA_CCR_EN;

		ibus_received = 1;

		if (I2C1_read_write_flag == 0) {
			EXTI->IMR |= EXTI_IMR_IM9;
		}
	}
}
void USART2_IRQHandler(void) {
	//RDR not empty flag:
	static double time_flag5_1;
	if (0 != (USART_ISR_RXNE & USART2->ISR)) {
		//check gap duration if bigger than 500 us brake

//tu jest coœ Ÿle przy zakomenntowanym dzia³a porawinie a jesli to odkomentujê przerwa wynosi oko³o 0.065[s] a to czas trwania 10 ramek ibusa bez sensu
//		if ( (get_Global_Time() - time_flag5_1) > 0.0005) {
//			rxindex = 0;
//		}
//
//		time_flag5_1 = get_Global_Time();

		//	read actual value of I-BUS (Interrupt flag will be automatically removed):
		rxBuf[rxindex] = USART2->RDR;
		if (rxindex == 1 && rxBuf[rxindex] == 0x40) {

			//block USART2 interrupt until DMA reading finish and data are processed:
			USART2->CR1 &= ~USART_CR1_RXNEIE;
			EXTI->IMR &= ~EXTI_IMR_IM9;

			//start DMA USART2 reading:
//			for (int i=0;i<300;i++){
//				;
//			}

//TO nie dzia³a

//			static int i;
//			while(USART_ISR_RXNE & USART2->ISR){
//
//			i=USART2->ISR;
//			}

// to tez nie :

//			if(USART_ISR_RXNE&USART2->ISR){
//			USART2->RQR|=USART_RQR_RXFRQ;
//			}

//to dzia³a

//delay_micro(90);//waiting for clearing RDR register FIND BETTER WAY!!!!

			DMA1_Channel5->CCR |= DMA_CCR_EN;

		} else if (rxindex == 0 && rxBuf[rxindex] != 0x20) {
			rxindex = 0;

		} else if (rxindex == 1 && rxBuf[rxindex] != 0x40) {
			rxindex = 0;

		} else {
			//if header is right increase rxindex
			rxindex++;
		}
	}
//idle detection flag:
	if (0 != (USART_ISR_IDLE & USART2->ISR)) {
		USART2->ICR |= USART_ICR_IDLECF;

		if (ibus_received == 0) {
			USART2->CR1 |= USART_CR1_RXNEIE;
		}
		pok1++;
	}
}

void Ibus_save() {
	static double time_flag3_1;
	if ((get_Global_Time() - time_flag3_1) >= MAX_NO_SIGNAL_TIME) {
		failsafe_type = 3;
		EXTI->SWIER |= EXTI_SWIER_SWI15;
	}

	// checking checksum and rewriting rxBuf to channels:
	if (ibus_received) {
		time_flag3_1 = get_Global_Time();
		uint16_t checksum = 0xFFFF;
		for (int8_t i = 0; i < 30; i++) {
			checksum -= rxBuf[i];
		}
		if (checksum == ((rxBuf[31] << 8) + rxBuf[30])) {
			for (int8_t i = 0; i < CHANNELS; i++) {
				channels[i] = (rxBuf[2 * (i + 1) + 1] << 8)
						+ rxBuf[2 * (i + 1)];
			}

			failsafe_RX();
			Throttle = channels[2];
		}

		//	unlock receiving new data from i-Bus:
		rxindex = 0;
		ibus_received = 0;
		USART2->CR1 |= USART_CR1_RXNEIE;
	}
}
static void failsafe_RX() {
	// Arming switch - SA
	if (channels[4] <= DISARM_VALUE) {
		failsafe_type = 1;
		EXTI->SWIER |= EXTI_SWIER_SWI15;
	} else if (channels[0] <= MIN_RX_SIGNAL || channels[0] >= MAX_RX_SIGNAL
			|| channels[1] <= MIN_RX_SIGNAL || channels[1] >= MAX_RX_SIGNAL
			|| channels[2] <= MIN_RX_SIGNAL || channels[2] >= MAX_RX_SIGNAL
			|| channels[3] <= MIN_RX_SIGNAL || channels[3] >= MAX_RX_SIGNAL) {

		failsafe_type = 2;
		EXTI->SWIER |= EXTI_SWIER_SWI15;

	} else {
		PWM_M1 = &pwm_m1;
		PWM_M2 = &pwm_m2;
		PWM_M3 = &pwm_m3;
		PWM_M4 = &pwm_m4;
	}
}
