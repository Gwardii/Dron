/*
 * conection.c
 *
 *  Created on: 17.03.2021
 *      Author: symon
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "connection.h"

extern volatile uint8_t time[];
extern uint8_t txDone;
extern volatile int32_t g_txSize;
extern volatile int32_t g_txTransmitted;

static uint8_t bufor[50];

void USART1_IRQHandler(void) {

	if (0 != (USART_ISR_RXNE & USART1->ISR)) {
		//	read actual value of I-BUS (flag will be automatically removed):
		static uint8_t i;

		bufor[i] = USART1->RDR;
		i++;
		if(i>=50){
			i=0;
		}
	}

//TRANSMISJA:

//	check if interrupt was generated by right flag:
	if ((0 != (USART_CR1_TXEIE & USART1->CR1))
			&& (0 != (USART_ISR_TXE & USART1->ISR))) {
		//	transmit data:
		if ((0 != g_txSize) && (g_txTransmitted < g_txSize)) {
			USART1->TDR = time[g_txTransmitted];
			g_txTransmitted += 1;
		}
		// if everything is transmitted, unable transmission interrupts:
		if (g_txTransmitted == g_txSize) {
			USART1->CR1 &= ~USART_CR1_TXEIE;
			txDone = 1;
		}
	}
}

