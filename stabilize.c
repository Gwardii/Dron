/*
 * stabilize.c
 *
 *  Created on: 04.01.2021
 *      Author: filip
 */
#include <math.h>
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "MPU6050.h"
#include "stabilize.h"

#define GYRO_PART 0.98
#define ACC_PART 0.02
#define GYRO_TO_DPS 32768/1000. // convert gyro register into degrees per second unit

#define GYRO_ROLL_OFFSET -28.787424166100877
#define GYRO_PITCH_OFFSET 23.64611577126087
#define GYRO_YAW_OFFSET -45.658468209991462

#define MAX_ROLL_ANGLE 10
#define MAX_PITCH_ANGLE 10


#define MEDIAN_BUFFOR 11

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
}PID;

typedef struct{
	int32_t roll;
	int32_t pitch;
	int32_t yaw;
}Three;

typedef struct{
	double roll;
	double pitch;
	double yaw;
}ThreeD;

static ThreeD angles;
static ThreeD gangles={0,0,0};

static const double rad_to_deg = 180 / atan(1) / 4;
static double acc_angle_roll;
static double acc_angle_pitch;
static double gyro_angle_roll;
static double gyro_angle_pitch;

static double dt;

static double milis();
static void gyro_angles(ThreeD*);
static void complementary_filter();
static void acc_angles();
static ThreeD median_filter(ThreeD *);
static ThreeD angles_PID();
static void set_motors(ThreeD);
static void anti_windup();

// err values - difference between set value and measured value:
static ThreeD err={0,0,0};
static ThreeD sum_err	=	{0, 0, 0};
static ThreeD last_err 	=	{0, 0, 0};
static ThreeD D_corr={0,0,0};
static ThreeD last_D_corr={0,0,0};
static Three Rates = { 700, 700, 400 };

static PID R_PID 	=	 {0,0,0.02};
static PID P_PID 	=	 {0,0,0.02};
static PID Y_PID 	=	 {0,0,0};

void stabilize(){

	static double timer;
	timer += milis();
	if(timer < 20){
		return;
	}
	dt = timer/1000.;
	timer = 0;
	// dryf zyroskopu:
	gyro_angles(&gangles);
	gyro_angle_roll=gangles.roll;
	gyro_angle_pitch=gangles.pitch;

	complementary_filter();
	set_motors(angles_PID());

	// wypisywanie katów gyro (roll pitch) acc(roll pitch) po komplementarnym (roll pitch)
	table_to_send[0]=0.1*(gyro_angle_roll*GYRO_TO_DPS+32768);
	table_to_send[1]=0.1*(gyro_angle_pitch*GYRO_TO_DPS+32768);
	table_to_send[2]=0.1*(acc_angle_roll*GYRO_TO_DPS+32768);
	table_to_send[3]=0.1*(acc_angle_pitch*GYRO_TO_DPS+32768);
	table_to_send[4]=0.1*(angles.roll*GYRO_TO_DPS+32768);
	table_to_send[5]=0.1*(angles.pitch*GYRO_TO_DPS+32768);

//	//err. Pitch Roll Yaw
//	table_to_send[0]=0.1*(err.pitch+32768);
//	table_to_send[1]=0.1*(err.roll+32768);
//	table_to_send[2]=0.1*(err.yaw+32768);


//	//wypisywanie korekcji pitch P I D i roll P I D
//	table_to_send[0]=R_PID.P*err.pitch*500./32768.+1000;
//	table_to_send[1]=P_PID.I*sum_err.pitch*500./32768.+1000;
//	table_to_send[2]=P_PID.D*D_corr.pitch*500./32768.+1000;
//	table_to_send[3]=P_PID.P*err.roll*500./32768.+1000;
//	table_to_send[4]=P_PID.I*sum_err.roll*500./32768.+1000;
//	table_to_send[5]=P_PID.D*D_corr.roll*500./32768.+1000;


}
 static void acc_angles(){
	static ThreeD acc_outcome[MEDIAN_BUFFOR];
	for(int i = MEDIAN_BUFFOR - 1; i > 0; i--){
		acc_outcome[i] = acc_outcome[i-1];
	}
	acc_outcome[0].pitch 	= 	Gyro_Acc[3];
	acc_outcome[0].roll 	= 	Gyro_Acc[4];
	acc_outcome[0].yaw 		= 	Gyro_Acc[5];
	ThreeD acc_filtered = median_filter(acc_outcome);
	acc_angle_roll 	= 	atan2(acc_filtered.roll, acc_filtered.yaw) * rad_to_deg+10.2;
	acc_angle_pitch	=	-atan2(acc_filtered.pitch, acc_filtered.yaw) * rad_to_deg+1.644;
	//atan2(-acc_filtered.roll, sqrt(acc_filtered.pitch*acc_filtered.pitch	+ acc_filtered.yaw*acc_filtered.yaw));
}

static double milis() {
	static uint16_t t1;
	double temp;
	uint16_t t2 = TIM2->CNT;
	if (t2 > t1) {
		temp = (t2 - t1) / 1000.;
	}
	else{
		temp = (TIM2->ARR + 1 + t2 - t1) / 1000.;
	}
	t1 = t2;
	return temp;
}

static void gyro_angles(ThreeD *gyro_angles){
	gyro_angles->roll 	+=	 (Gyro_Acc[0] - GYRO_ROLL_OFFSET) * dt/(GYRO_TO_DPS);
	gyro_angles->pitch 	+=	 (Gyro_Acc[1] - GYRO_PITCH_OFFSET) * dt/(GYRO_TO_DPS);
	//gyro_angles->yaw 	+=	 GYRO_TO_DPS * (Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt;
}

static void complementary_filter(){
	gyro_angles(&angles);
	acc_angles();
	angles.roll			=	ACC_PART * acc_angle_roll + GYRO_PART * angles.roll;
	angles.pitch		=	ACC_PART * acc_angle_pitch + GYRO_PART * angles.pitch;

}

static ThreeD median_filter(ThreeD values[]){
	ThreeD filtered_values = values[0];
	ThreeD count;
	for(int i = 0; i < MEDIAN_BUFFOR; i++){
		count.roll = 0;
		count.pitch = 0;
		count.yaw = 0;
		for(int j = 0; j < i; j++){
			if(values[i].roll <= values[j].roll){
				count.roll += 1;
			}
			if(values[i].pitch <= values[j].pitch){
				count.pitch += 1;
			}
			if(values[i].yaw <= values[j].yaw){
				count.yaw += 1;
			}
			}
		for(int j = i+1; j < MEDIAN_BUFFOR; j++){
			if(values[i].roll <= values[j].roll){
				count.roll += 1;
			}
			if(values[i].pitch <= values[j].pitch){
				count.pitch += 1;
			}
			if(values[i].yaw <= values[j].yaw){
				count.yaw += 1;
			}
			}
		if(count.roll == MEDIAN_BUFFOR / 2){
			filtered_values.roll = values[i].roll;
		}
		if(count.pitch == MEDIAN_BUFFOR / 2){
			filtered_values.pitch = values[i].pitch;
		}
		if(count.yaw == MEDIAN_BUFFOR / 2){
			filtered_values.yaw = values[i].yaw;
		}	
		}

	return filtered_values;
}

static ThreeD angles_PID(){
	ThreeD corr;

	err.roll	=	((channels[0] - 1500)*32768/500. - angles.roll*32768/MAX_ROLL_ANGLE);
	err.pitch	=	((channels[1] - 1500)*32768/500. - angles.pitch*32768/MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500) * 32768/500. - Gyro_Acc[2] * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll 	+=	 err.roll*dt;
	sum_err.pitch 	+=	 err.pitch*dt;
	sum_err.yaw		+=	 err.yaw*dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;

	D_corr.roll= (err.roll-last_err.roll)/dt;
	D_corr.pitch=(err.pitch-last_err.pitch)/dt;
	D_corr.yaw=(err.yaw-last_err.yaw)/dt;

	anti_windup();

	//	calculate corrections:
	corr.roll	=	(R_PID.P * err.roll + R_PID.I*sum_err.roll + R_PID.D *D_corr.roll)*500/ 32768.;
	corr.pitch	=	(P_PID.P * err.pitch + P_PID.I*sum_err.pitch + P_PID.D * D_corr.pitch)*500/ 32768.;
	corr.yaw = (Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw+Y_PID.D*D_corr.yaw)*500/ 32768.;

		//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;

	last_D_corr.roll=D_corr.roll;
	last_D_corr.pitch=D_corr.pitch;
	last_D_corr.yaw=D_corr.yaw;

	return corr;
}
static void anti_windup() {
	int16_t max_I_correction = 300;
	if ((sum_err.roll * R_PID.I*500./32768.) > max_I_correction) {
		sum_err.roll = max_I_correction / R_PID.I/500.*32768.;
	} else if ((sum_err.roll * R_PID.I*500./32768.) < -max_I_correction) {
		sum_err.roll = -max_I_correction / R_PID.I/500.*32768.;
	}
	if ((sum_err.pitch * P_PID.I*500./32768.) > max_I_correction) {
		sum_err.pitch = max_I_correction / P_PID.I/500.*32768.;
	} else if ((sum_err.pitch * P_PID.I*500/32768.) < -max_I_correction) {
		sum_err.pitch = -max_I_correction / P_PID.I/500.*32768.;
	}
	if ((sum_err.yaw * Y_PID.I*500./32768.) > max_I_correction) {
		sum_err.yaw = max_I_correction / Y_PID.I/500.*32768.;
	} else if ((sum_err.yaw * Y_PID.I*500/32768. )< -max_I_correction) {
		sum_err.yaw = -max_I_correction / Y_PID.I/500.*32768.;
	}
	int16_t max_D_correction = 300;
	if((D_corr.roll*R_PID.D*500./32768.)>max_D_correction||(D_corr.roll*R_PID.D*500./32768.)<-max_D_correction){
		D_corr.roll=last_D_corr.roll;
	}
	if(D_corr.pitch*P_PID.D*500./32768.>max_D_correction||D_corr.pitch*P_PID.D*500./32768.<-max_D_correction){
		D_corr.pitch=last_D_corr.pitch;
	}
	if(D_corr.yaw*Y_PID.D*500./32768.>max_D_correction||D_corr.yaw*Y_PID.D*500./32768.<-max_D_correction){
		D_corr.yaw=last_D_corr.yaw;
	}


}
static void set_motors(ThreeD corr){
	//	Make corrections:

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
}
