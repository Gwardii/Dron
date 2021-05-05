/*
 * connection.h
 *
 *  Created on: 17.03.2021
 *      Author: symon
 */

#ifndef CONNECTION_H_
#define CONNECTION_H_

void USART1_IRQHandler();
void DMA1_Channel2_3_IRQHandler();
void print(uint16_t x[], uint8_t data_to_send);

#endif /* CONNECTION_H_ */
