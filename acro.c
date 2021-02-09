/*
 * acro.c
 *
 *  Created on: 06.02.2021
 *      Author: symon
 */

#include <math.h>
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "MPU6050.h"
#include "acro.h"

static uint16_t micros();
void anti_windup();

#define GYRO_ROLL_OFFSET 0
#define GYRO_PITCH_OFFSET 0
#define GYRO_YAW_OFFSET 0

#define MEDIAN_BUFFOR 5

extern int16_t Gyro_Acc[];
extern int16_t channels[];
extern int16_t Throttle;
extern uint16_t PWM_M1;
extern uint16_t PWM_M2;
extern uint16_t PWM_M3;
extern uint16_t PWM_M4;

typedef struct {
	double P;
	double I;
	double D;
} PID;

typedef struct {
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
} Three;

typedef struct {
	double roll;
	double pitch;
	double yaw;
} ThreeD;

//---User defines maximum speed of spinning [deg/s]----

Three Rates = { 700, 700, 500 };

PID R_PID = { 0.5, 0.3, 0 };
PID P_PID = { 0.5, 0.3, 0 };
PID Y_PID = { 0.65, 0.3, 0 };

Three err;
Three sum_err = { 0, 0, 0 };

void acro() {
	gyro_read();
	Three corr;


	err.roll = (channels[0] - 1500) * 32768/1000 - Gyro_Acc[0] * 1000 / Rates.roll;
	err.pitch = (channels[1] - 1500) * 32768/1000 - Gyro_Acc[1] * 1000 / Rates.pitch;
	err.yaw = (channels[3] - 1500) * 32768/1000 - Gyro_Acc[2] * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll;
	sum_err.pitch += err.pitch;
	sum_err.yaw += err.yaw;

	anti_windup();

	corr.pitch = (P_PID.P * err.pitch + P_PID.I*sum_err.pitch)*1000/ 32768;
	corr.roll = (R_PID.P * err.roll + R_PID.I*sum_err.roll)*1000/ 32768;
	corr.yaw = (Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw)*1000/ 32768;

//	right front:
	PWM_M1 = Throttle - corr.pitch + corr.yaw - corr.roll;

	//	right back:
	PWM_M2 = Throttle + corr.pitch - corr.yaw - corr.roll;

	//	left back:
	PWM_M3 = Throttle + corr.pitch + corr.yaw + corr.roll;

	//	left front:
	PWM_M4 = Throttle - corr.pitch - corr.yaw + corr.roll;

	if (PWM_M1 < 1050) {
		PWM_M1 = 1050;
	}
	if (PWM_M2 < 1050) {
		PWM_M2 = 1050;
	}
	if (PWM_M3 < 1050) {
		PWM_M3 = 1050;
	}
	if (PWM_M4 < 1050) {
		PWM_M4 = 1050;
	}
}

void anti_windup() {
	int16_t max_I_correction = 300;
	if (sum_err.roll * R_PID.I > max_I_correction) {
		sum_err.roll = max_I_correction / R_PID.I;
	} else if (sum_err.roll * R_PID.I < -max_I_correction) {
		sum_err.roll = -max_I_correction / R_PID.I;
	}
	if (sum_err.pitch * P_PID.I > max_I_correction) {
		sum_err.pitch = max_I_correction / P_PID.I;
	} else if (sum_err.pitch * P_PID.I < -max_I_correction) {
		sum_err.pitch = -max_I_correction / P_PID.I;
	}
	if (sum_err.yaw * Y_PID.I > max_I_correction) {
		sum_err.yaw = max_I_correction / Y_PID.I;
	} else if (sum_err.yaw * Y_PID.I < -max_I_correction) {
		sum_err.yaw = -max_I_correction / Y_PID.I;
	}

}

static uint16_t micros() {
	static uint16_t t1;
	uint16_t t2 = TIM2->CNT;
	if (t2 > t1) {
		return t2 - t1;
	}
	return (uint16_t) TIM2->ARR + 1 + t2 - t1;
}

