/*
 * global_functions.c

 *
 *  Created on: 03.05.2021
 *      Author: symon
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_functions.h"

double Global_Time = 0;

void delay_micro(uint16_t time) {
	Global_Time += (TIM6->CNT) / 1000000.;
	TIM6->CNT = 0;
	while (TIM6->CNT < time) {
		; //wait for time [us]
	}
}
void delay_mili(uint16_t time) {
//wait for time [ms]
	for (uint16_t i = 0; i < time; i++) {
		delay_micro(1000);
	}
}
double timer(double t1) {

	double delta_time;
	double t2 = (TIM2->CNT) / 1000000.;
	delta_time = t2 - t1;
	if (delta_time < 0) {
		delta_time += (double) (TIM2->ARR) / 1000000.;
	}
	return delta_time;
}
void TIM6_DAC_IRQHandler() {
	if (TIM_SR_UIF & TIM6->SR) {
		TIM6->SR &= ~TIM_SR_UIF;
		Global_Time += (TIM6->ARR) / 1000000.;
	}
}
double get_Global_Time() {
	return (Global_Time + (TIM6->CNT) / 1000000.);
}
