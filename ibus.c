/*
 * ibus.c
 *
 *  Created on: 25.01.2021
 *      Author: filip
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "ibus.h"

volatile int32_t g_txSize = 0;
volatile int32_t g_txTransmitted = 0;
volatile int8_t dataFlag = 0;
volatile int8_t dataFlag2 = 0;
extern uint16_t current_time;
extern uint16_t last_time;

volatile uint8_t rxBuf[32];
volatile uint8_t rxindex = 0;
volatile int8_t ibus_received = 0;
int8_t new_I_Bus = 0;

extern volatile uint8_t time[];
extern uint8_t CHANNELS;
extern uint16_t channels[];
extern uint8_t gap_time;
extern uint8_t txDone;

void USART2_IRQHandler(void) {

	//ODBIOR OD RX:

	//	check if interrupt was generated by right flag:
	if (0 != (USART_ISR_RXNE & USART2->ISR)) {
		//	read actual value of I-BUS (flag will be automatically removed):

		current_time = TIM21->CNT;
		if (current_time < last_time) {
			gap_time = current_time - last_time + TIM2->ARR;
		} else {
			gap_time = current_time - last_time;
		}
		last_time = current_time;
		if (gap_time > 50) {
			rxindex = 0;
		}
		rxBuf[rxindex] = USART2->RDR;
		if (rxindex == 31) {
			ibus_received = 1;
			//block receiving new data until old data are processed:
			USART2->CR1 &= ~USART_CR1_RXNEIE;
		}	//	waiting for header 0x2040:
		else if (rxindex == 0 && rxBuf[rxindex] != 0x20) {
			rxindex = 0;
		} else if (rxindex == 1 && rxBuf[rxindex] != 0x40) {
			rxindex = 0;
		} else {	//if header is right increase rxindex
			rxindex++;
		}
	}

//TRANSMISJA:

	//	check if interrupt was generated by right flag:
	if ((0 != (USART_CR1_TXEIE & USART2->CR1))
			&& (0 != (USART_ISR_TXE & USART2->ISR))) {
		//	transmit data:
		if ((0 != g_txSize) && (g_txTransmitted < g_txSize)) {
			USART2->TDR = (uint32_t) time[g_txTransmitted];
			g_txTransmitted += 1;
		}
		// if everything is transmitted, unable transmission interrupts:
		if (g_txTransmitted == g_txSize) {
			USART2->CR1 &= ~USART_CR1_TXEIE;
			txDone = 1;
		}
	}
}

void Ibus_save() {
	// checking checksum and rewriting rxBuf to channels:
	if (ibus_received) {
		uint16_t checksum = 0xFFFF;
		for (int8_t i = 0; i < 30; i++) {
			checksum -= rxBuf[i];
		}
		if (checksum == ((rxBuf[31] << 8) + rxBuf[30])) {
			for (int8_t i = 0; i < CHANNELS; i++) {
				channels[i] = (rxBuf[2 * (i + 1) + 1] << 8)
						+ rxBuf[2 * (i + 1)];
			}
			new_I_Bus = 1;
			rxindex = 0;
		}
		//	unlock receiving data from i-Bus:
		ibus_received = 0;
		rxindex = 0;
		USART2->CR1 |= USART_CR1_RXNEIE;
	}
}
