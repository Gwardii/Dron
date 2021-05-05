/*
 * stabilize.c
 *s
 */
#include <math.h>
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "MPU6050.h"
#include "stabilize.h"



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

static ThreeD angles;
static ThreeD gangles = { 0, 0, 0 };

static const double rad_to_deg = 180 / atan(1) / 4;
static double acc_angle_roll = 0;
static double acc_angle_pitch = 0;
static double gyro_angle_roll = 0;
static double gyro_angle_pitch = 0;

static double dt;


static void gyro_angles(ThreeD*);
static void complementary_filter();
static void acc_angles();
static ThreeD corrections();
static void set_motors(ThreeD);
static void anti_windup();


// err values - difference between set value and measured value:
static ThreeD err = { 0, 0, 0 };
static ThreeD sum_err = { 0, 0, 0 };
static ThreeD last_err = { 0, 0, 0 };
static ThreeD D_corr = { 0, 0, 0 };
static ThreeD last_D_corr = { 0, 0, 0 };
static Three Rates = { 400, 400, 400 };

static PID R_PID = { 0.09, 0.025, 0.015 };
static PID P_PID = { 0.09, 0.025, 0.015 };
static PID Y_PID = { 2, 0.1, 0.0005 };

//for debugging only:
static int puk2=0;

void stabilize() {

	static double time_flag1_1;
	static double time_flag1_2;

	dt = (get_Global_Time()-time_flag1_1);
	time_flag1_1=get_Global_Time();


	//median_filter(measurement_tab);
	// dryf zyroskopu:
	gyro_angles(&gangles);
	gyro_angle_roll = gangles.roll;
	gyro_angle_pitch = gangles.pitch;

	complementary_filter();
	set_motors(corrections());


	if ((get_Global_Time()-time_flag1_2) >= 1. / FREQUENCY_TELEMETRY_UPDATE) {
		time_flag1_2=get_Global_Time();
		puk2++;
		//wypisywanie korekcji pitch P I D i roll P I D; k¹tów; zadanych wartosci
		table_to_send[0] = P_PID.P * err.pitch * 500. / 32768. + 1000;
		table_to_send[1] = P_PID.I * sum_err.pitch * 500. / 32768. + 1000;
		table_to_send[2] = P_PID.D * D_corr.pitch * 500. / 32768. + 1000;
		table_to_send[3] = R_PID.P * err.roll * 500. / 32768. + 1000;
		table_to_send[4] = R_PID.I * sum_err.roll * 500. / 32768. + 1000;
		table_to_send[5] = R_PID.D * D_corr.roll * 500. / 32768. + 1000;
		table_to_send[6] = (angles.roll / MAX_ROLL_ANGLE * 50) + 1000;
		table_to_send[7] = (angles.pitch / MAX_PITCH_ANGLE * 50) + 1000;
		table_to_send[8] = 10 * (gyro_angle_roll + 360);
		table_to_send[9] = 10 * (gyro_angle_pitch + 360);
		table_to_send[10] = 10 * (acc_angle_roll + 360);
		table_to_send[11] = 10 * (acc_angle_pitch + 360);
		table_to_send[12] = channels[1] - 500;
		table_to_send[13] = channels[0] - 500;

		New_data_to_send = 1;

	}
}

static void acc_angles() {
double acc_filter_rate=0.05;
	acc_angle_roll = (1-acc_filter_rate)*acc_angle_roll+acc_filter_rate*(atan2(Gyro_Acc[4], Gyro_Acc[5]) * rad_to_deg + 8.16);
	acc_angle_pitch = (1-acc_filter_rate)*acc_angle_pitch+acc_filter_rate*(-atan2(Gyro_Acc[3], Gyro_Acc[5]) * rad_to_deg - 1);
}

static void gyro_angles(ThreeD *gyro_angles) {
	gyro_angles->roll += (Gyro_Acc[0] - GYRO_ROLL_OFFSET) * dt / (GYRO_TO_DPS);
	gyro_angles->pitch += (Gyro_Acc[1] - GYRO_PITCH_OFFSET) * dt
			/ (GYRO_TO_DPS);
	gyro_angles->roll += gyro_angles->pitch
			* sin(
					(Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt / (GYRO_TO_DPS)
							/ rad_to_deg);
	gyro_angles->pitch -= gyro_angles->roll
			* sin(
					(Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt / (GYRO_TO_DPS)
							/ rad_to_deg);

}
static void complementary_filter() {
	gyro_angles(&angles);
	acc_angles();
	angles.roll = ACC_PART * acc_angle_roll + GYRO_PART * angles.roll;
	angles.pitch = ACC_PART * acc_angle_pitch + GYRO_PART * angles.pitch;
}
static ThreeD corrections() {
	static ThreeD corr;
	static ThreeD last_channels;
	err.roll = ((channels[0] - 1500) * 32768 / 500.
			- angles.roll * 32768 / MAX_ROLL_ANGLE);
	err.pitch = ((channels[1] - 1500) * 32768 / 500.
			- angles.pitch * 32768 / MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500) * 32768 / 500.
			- (Gyro_Acc[2] - GYRO_YAW_OFFSET) * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll * dt;
	sum_err.pitch += err.pitch * dt;
	sum_err.yaw += err.yaw * dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;


	D_corr.roll = -(Gyro_Acc[0] - GYRO_ROLL_OFFSET) * 1000 / MAX_ROLL_ANGLE
			+ (channels[0] - last_channels.roll) / 500. * 32768 / dt;
	D_corr.pitch = -(Gyro_Acc[1] - GYRO_PITCH_OFFSET) * 1000 / MAX_PITCH_ANGLE
			+ (channels[1] - last_channels.pitch) / 500. * 32768 / dt;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	anti_windup();

	//	calculate corrections:
	corr.roll = (R_PID.P * err.roll + R_PID.I * sum_err.roll
			+ R_PID.D * D_corr.roll) * 500 / 32768.;
	corr.pitch = (P_PID.P * err.pitch + P_PID.I * sum_err.pitch
			+ P_PID.D * D_corr.pitch) * 500 / 32768.;
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

	last_channels.roll = channels[0];
	last_channels.pitch = channels[1];
	last_channels.yaw = channels[2];

	return corr;
}
static void anti_windup() {
	if (channels[4] > 1600) {
		int16_t max_I_correction = 300;
		if ((sum_err.roll * R_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.roll = max_I_correction / R_PID.I / 500. * 32768.;
		} else if ((sum_err.roll * R_PID.I * 500. / 32768.)
				< -max_I_correction) {
			sum_err.roll = -max_I_correction / R_PID.I / 500. * 32768.;
		}
		if ((sum_err.pitch * P_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.pitch = max_I_correction / P_PID.I / 500. * 32768.;
		} else if ((sum_err.pitch * P_PID.I * 500 / 32768.)
				< -max_I_correction) {
			sum_err.pitch = -max_I_correction / P_PID.I / 500. * 32768.;
		}
		if ((sum_err.yaw * Y_PID.I * 500. / 32768.) > max_I_correction) {
			sum_err.yaw = max_I_correction / Y_PID.I / 500. * 32768.;
		} else if ((sum_err.yaw * Y_PID.I * 500 / 32768.) < -max_I_correction) {
			sum_err.yaw = -max_I_correction / Y_PID.I / 500. * 32768.;
		}
	} else {			// quad is disarmed so turn off I term of corrections
		sum_err.roll = 0;
		sum_err.pitch = 0;
		sum_err.yaw = 0;
	}

	int16_t max_D_correction = 300;
	if ((D_corr.roll * R_PID.D * 500. / 32768.) > max_D_correction
			|| (D_corr.roll * R_PID.D * 500. / 32768.) < -max_D_correction) {
		D_corr.roll = last_D_corr.roll;
	}
	if (D_corr.pitch * P_PID.D * 500. / 32768. > max_D_correction
			|| D_corr.pitch * P_PID.D * 500. / 32768. < -max_D_correction) {
		D_corr.pitch = last_D_corr.pitch;
	}
	if (D_corr.yaw * Y_PID.D * 500. / 32768. > max_D_correction
			|| D_corr.yaw * Y_PID.D * 500. / 32768. < -max_D_correction) {
		D_corr.yaw = last_D_corr.yaw;
	}

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
