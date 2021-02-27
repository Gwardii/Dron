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

static double timer();
static void anti_windup();

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

extern uint16_t table_to_send[];

typedef struct {
	double P;
	double I;
	double D;
}  PID;

typedef struct {
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
}  Three;

typedef struct {
	double roll;
	double pitch;
	double yaw;
}  ThreeD;

//---User defines maximum speed of spinning [deg/s]----

static Three Rates = { 500, 500, 400 };

static PID R_PID = { 0.6, 0.8, 0.013 };
static PID P_PID = { 0.6, 0.8, 0.013};
static PID Y_PID = { 3, 0.05, 0.005};

static Three err={0,0,0};
static ThreeD sum_err = { 0, 0, 0 };
static Three last_err = {0,0,0};
static Three D_corr={0,0,0};
static Three last_D_corr={0,0,0};

static double dt;

 void acro() {
	Three corr={0,0,0};

	static double tim;
	tim += timer();
	if (tim < 0.02) {
		return;
	}
	dt = tim;
	tim = 0;

	err.roll = (channels[0] - 1500) * 32768/500. - Gyro_Acc[0] * 1000 / Rates.roll;
	err.pitch = (channels[1] - 1500) * 32768/500. - Gyro_Acc[1] * 1000 / Rates.pitch;
	err.yaw = (channels[3] - 1500) * 32768/500. - Gyro_Acc[2] * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll += err.roll*dt;
	sum_err.pitch += err.pitch*dt;
	sum_err.yaw += err.yaw*dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;

	D_corr.roll = (err.roll - last_err.roll) / dt;
	D_corr.pitch = (err.pitch - last_err.pitch) / dt;
	D_corr.yaw = (err.yaw - last_err.yaw) / dt;

	anti_windup();

	//	calculate corrections:
	corr.pitch = (P_PID.P * err.pitch + P_PID.I*sum_err.pitch+P_PID.D*D_corr.pitch)*500/ 32768.;
	corr.roll = (R_PID.P * err.roll + R_PID.I*sum_err.roll+R_PID.D*D_corr.roll)*500/ 32768.;
	corr.yaw = (Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw+Y_PID.D*D_corr.yaw)*500/ 32768.;

	//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;

	last_D_corr.roll=D_corr.roll;
	last_D_corr.pitch=D_corr.pitch;
	last_D_corr.yaw=D_corr.yaw;

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
	else if(PWM_M1 > 2000)
		PWM_M1 = 2000;

	if (PWM_M2 < 1050) {
		PWM_M2 = 1050;
	}
	else if(PWM_M2 > 2000)
		PWM_M2 = 2000;

	if (PWM_M3 < 1050) {
		PWM_M3 = 1050;
	}
	else if(PWM_M3 > 2000)
		PWM_M3 = 2000;

	if (PWM_M4 < 1050) {
		PWM_M4 = 1050;
	}
	else if(PWM_M4 > 2000)
		PWM_M4 = 2000;


//		//err. Pitch Roll Yaw
//		table_to_send[0]=0.1*(err.pitch+32768);
//		table_to_send[1]=0.1*(err.roll+32768);
//		table_to_send[2]=0.1*(err.yaw+32768);


		//wypisywanie korekcji pitch P I D i roll P I D
		table_to_send[0]=R_PID.P*err.pitch*500./32768.+1000;
		table_to_send[1]=P_PID.I*sum_err.pitch*500./32768.+1000;
		table_to_send[2]=P_PID.D*D_corr.pitch*500./32768.+1000;
		table_to_send[3]=P_PID.P*err.roll*500./32768.+1000;
		table_to_send[4]=P_PID.I*sum_err.roll*500./32768.+1000;
		table_to_send[5]=P_PID.D*D_corr.roll*500./32768.+1000;

}

static void anti_windup() {
	int16_t max_I_correction = 300;
	if ((sum_err.roll * R_PID.I*500/32768.) > max_I_correction) {
		sum_err.roll = max_I_correction / R_PID.I/500.*32768.;
	} else if ((sum_err.roll * R_PID.I*500/32768.) < -max_I_correction) {
		sum_err.roll = -max_I_correction / R_PID.I/500.*32768.;
	}
	if ((sum_err.pitch * P_PID.I*500/32768.) > max_I_correction) {
		sum_err.pitch = max_I_correction / P_PID.I/500.*32768.;
	} else if ((sum_err.pitch * P_PID.I*500/32768.) < -max_I_correction) {
		sum_err.pitch = -max_I_correction / P_PID.I/500.*32768.;
	}
	if ((sum_err.yaw * Y_PID.I*500/32768.) > max_I_correction) {
		sum_err.yaw = max_I_correction / Y_PID.I/500.*32768.;
	} else if ((sum_err.yaw * Y_PID.I*500/32768. )< -max_I_correction) {
		sum_err.yaw = -max_I_correction / Y_PID.I/500.*32768.;
	}
	int16_t max_D_correction = 300;
	if((D_corr.roll*R_PID.D*500/32768.)>max_D_correction||(D_corr.roll*R_PID.D*500/32768.)<-max_D_correction){
		D_corr.roll=last_D_corr.roll;
	}
	if(D_corr.pitch*P_PID.D*500/32768.>max_D_correction||D_corr.pitch*P_PID.D*500/32768.<-max_D_correction){
		D_corr.pitch=last_D_corr.pitch;
	}
	if(D_corr.yaw*Y_PID.D*500/32768.>max_D_correction||D_corr.yaw*Y_PID.D*500/32768.<-max_D_correction){
		D_corr.yaw=last_D_corr.yaw;
	}


}

static double timer(){
	static uint16_t t1;
	double temp;
	uint16_t t2 = TIM2->CNT;
	if(t2 > t1){
		temp = (t2 - t1)/1000000.;
	}
	else{
		temp = (TIM2->ARR + 1 + t2 - t1)/1000000.;
	}
	t1=t2;
	return temp;
}
