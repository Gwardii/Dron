/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"
#include "MPU6050.h"
#include "stabilize.h"
#include "acro.h"
#include "ibus.h"
#include "connection.h"


void update_motors();

//for debugging only:
static int pik = 0;
static int pek = 0;

int main(void) {

	setup();
	setup_MPU6050();
	setup_NVIC();

	static double time_flag0_1=0;
	static double time_flag0_2=0;
	static double time_flag0_3=0;
	while (1) {

		if ((get_Global_Time()-time_flag0_1) >= 10) {
			time_flag0_1 = get_Global_Time();
			pik = 0;
			pek = 0;
		}
		pek++;

		if ((get_Global_Time()-time_flag0_2)>= 1. / FREQUENCY_PID_LOOP) {
			time_flag0_2 = get_Global_Time();
			Ibus_save();
			rewrite_data();
			pik++;

			if (channels[6] < 1400) {
				acro();
			}
			if (channels[6] > 1450) {
				stabilize();
			}
			if (0 != transmitting_is_Done && 0 != New_data_to_send) {
				// Transmit data
				print(table_to_send, ALL_ELEMENTS_TO_SEND);
			}
		}

		if ((get_Global_Time()-time_flag0_3) >= 1. / FREQUENCY_ESC_UPDATE) {
			time_flag0_3=get_Global_Time();
			update_motors();

		}

	}
}

void update_motors() {
	TIM2->CCR1 = *PWM_M1 - 1; 			//wypelneinie motor 1
	TIM2->CCR2 = *PWM_M2 - 1; 			//wypelneinie motor 2
	TIM2->CCR3 = *PWM_M3 - 1; 			//wypelneinie motor 3
	TIM2->CCR4 = *PWM_M4 - 1; 			//wypelneinie motor 4
}



