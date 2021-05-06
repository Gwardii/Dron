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
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"

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

static void anti_windup();
static ThreeD corrections();
static void set_motors(ThreeD corr);

//---User defines maximum speed of spinning [deg/s]:
static Three Rates = { 500, 500, 400 };

static PID R_PID = { 0.6, 0.8, 0.013 };
static PID P_PID = { 0.6, 0.8, 0.013 };
static PID Y_PID = { 2, 0.1, 0.001 };

static Three err = { 0, 0, 0 };
static ThreeD sum_err = { 0, 0, 0 };
static Three last_err = { 0, 0, 0 };
static Three D_corr = { 0, 0, 0 };
static Three last_D_corr = { 0, 0, 0 };

static double dt;

//for debuging only:
static int puk2 = 0;
static double czas_trwania_petli = 0;

void acro() {
	czas_trwania_petli =get_Global_Time();
	static double time_flag2_1;
	static double time_flag2_2;

	dt = (get_Global_Time() - time_flag2_1);
	time_flag2_1 = get_Global_Time();

	set_motors(corrections());

	if ((get_Global_Time() - time_flag2_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
		time_flag2_2 = get_Global_Time();
		puk2++;
		//wypisywanie korekcji pitch P I D i roll P I D; k¹tów; zadanych wartosci
		table_to_send[0] = P_PID.P * err.pitch * 500. / 32768. + 1000;
		table_to_send[1] = P_PID.I * sum_err.pitch * 500. / 32768. + 1000;
		table_to_send[2] = P_PID.D * D_corr.pitch * 500. / 32768. + 1000;
		table_to_send[3] = R_PID.P * err.roll * 500. / 32768. + 1000;
		table_to_send[4] = R_PID.I * sum_err.roll * 500. / 32768. + 1000;
		table_to_send[5] = R_PID.D * D_corr.roll * 500. / 32768. + 1000;
		table_to_send[6] = (Gyro_Acc[0] / Rates.roll * 50) + 1000;
		table_to_send[7] = (Gyro_Acc[1] / Rates.pitch * 50) + 1000;
		table_to_send[8] = 0;
		table_to_send[9] = 0;
		table_to_send[10] = 0;
		table_to_send[11] = 0;
		table_to_send[12] = channels[1] - 500;
		table_to_send[13] = channels[0] - 500;


		czas_trwania_petli =get_Global_Time()-czas_trwania_petli;
		New_data_to_send = 1;
	}

}

static void anti_windup() {
	if (channels[4] > 1600) {
		int16_t max_I_correction = 300;
		if ((sum_err.roll * R_PID.I * 500 / 32768.) > max_I_correction) {
			sum_err.roll = max_I_correction / R_PID.I / 500. * 32768.;
		} else if ((sum_err.roll * R_PID.I * 500 / 32768.)
				< -max_I_correction) {
			sum_err.roll = -max_I_correction / R_PID.I / 500. * 32768.;
		}
		if ((sum_err.pitch * P_PID.I * 500 / 32768.) > max_I_correction) {
			sum_err.pitch = max_I_correction / P_PID.I / 500. * 32768.;
		} else if ((sum_err.pitch * P_PID.I * 500 / 32768.)
				< -max_I_correction) {
			sum_err.pitch = -max_I_correction / P_PID.I / 500. * 32768.;
		}
		if ((sum_err.yaw * Y_PID.I * 500 / 32768.) > max_I_correction) {
			sum_err.yaw = max_I_correction / Y_PID.I / 500. * 32768.;
		} else if ((sum_err.yaw * Y_PID.I * 500 / 32768.) < -max_I_correction) {
			sum_err.yaw = -max_I_correction / Y_PID.I / 500. * 32768.;
		}
	}

	else {			// quad is disarmed so turn off I term of corrections
		sum_err.roll = 0;
		sum_err.pitch = 0;
		sum_err.yaw = 0;
	}
	int16_t max_D_correction = 300;
	if ((D_corr.roll * R_PID.D * 500 / 32768.) > max_D_correction
			|| (D_corr.roll * R_PID.D * 500 / 32768.) < -max_D_correction) {
		D_corr.roll = last_D_corr.roll;
	}
	if (D_corr.pitch * P_PID.D * 500 / 32768. > max_D_correction
			|| D_corr.pitch * P_PID.D * 500 / 32768. < -max_D_correction) {
		D_corr.pitch = last_D_corr.pitch;
	}
	if (D_corr.yaw * Y_PID.D * 500 / 32768. > max_D_correction
			|| D_corr.yaw * Y_PID.D * 500 / 32768. < -max_D_correction) {
		D_corr.yaw = last_D_corr.yaw;
	}

}

static ThreeD corrections() {

	static ThreeD corr = { 0, 0, 0 };

	err.roll = (channels[0] - 1500) * 32768 / 500.
			- Gyro_Acc[0] * 1000 / Rates.roll;
	err.pitch = channels[1]  * 32768 / 500.
			- Gyro_Acc[1] * 1000 / Rates.pitch;
	err.yaw = (channels[3] - 1500) * 32768 / 500.
			- Gyro_Acc[2]  * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;

	D_corr.roll = (err.roll - last_err.roll) / dt;
	D_corr.pitch = (err.pitch - last_err.pitch) / dt;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	anti_windup();

	//	calculate corrections:
	corr.pitch = (P_PID.P * err.pitch + P_PID.I * sum_err.pitch
			+ P_PID.D * D_corr.pitch) * 500 / 32768.;
	corr.roll = (R_PID.P * err.roll + R_PID.I * sum_err.roll
			+ R_PID.D * D_corr.roll) * 500 / 32768.;
	corr.yaw =
			(Y_PID.P * err.yaw + Y_PID.I * sum_err.yaw + Y_PID.D * D_corr.yaw)
					* 500 / 32768.;

	//	set current errors as last errors:
	last_err.roll = err.roll;
	last_err.pitch = err.pitch;
	last_err.yaw = err.yaw;

	last_D_corr.roll = D_corr.roll;
	last_D_corr.pitch = D_corr.pitch;
	last_D_corr.yaw = D_corr.yaw;

	return corr;
}
static void set_motors(ThreeD corr) {
	//	Make corrections:
	//	right front:
	pwm_m1 = Throttle - corr.pitch + corr.yaw - corr.roll;
	//	right back:
	pwm_m2 = Throttle + corr.pitch - corr.yaw - corr.roll;
	//	left back:
	pwm_m3 = Throttle + corr.pitch + corr.yaw + corr.roll;
	//	left front:
	pwm_m4 = Throttle - corr.pitch - corr.yaw + corr.roll;
	if (pwm_m1 < 1050) {
		pwm_m1 = 1050;
	} else if (pwm_m1 > 2000)
		pwm_m1 = 2000;
	if (pwm_m2 < 1050) {
		pwm_m2 = 1050;
	} else if (pwm_m2 > 2000)
		pwm_m2 = 2000;
	if (pwm_m3 < 1050) {
		pwm_m3 = 1050;
	} else if (pwm_m3 > 2000)
		pwm_m3 = 2000;
	if (pwm_m4 < 1050) {
		pwm_m4 = 1050;
	} else if (pwm_m4 > 2000)
		pwm_m4 = 2000;

}

