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

#define GYRO_PART (995 / 32768)
#define ACC_PART 5

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
	uint8_t P;
	uint8_t I;
	uint8_t D;
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

static const double rad_to_deg = atan(1) * 4;
static double acc_angle_roll;
static double acc_angle_pitch;

static int16_t dt;

static uint16_t micros();
static void gyro_angles(ThreeD*);
static void acc_angles();
static void complementary_filter();
static Three median_filter(Three *);
static Three rates_PID();
static Three angles_PID();

void stabilize(){
	gyro_read();
	micros();
	acc_read();
	complementary_filter();
	Three corr = angles_PID(rates_PID());
	//	Make corrections:

	//	right front:
	PWM_M1 = Throttle - corr.pitch + corr.yaw + corr.roll;
	//	right back:
	PWM_M2 = Throttle + corr.pitch - corr.yaw + corr.roll;
	//	left back:
	PWM_M3 = Throttle + corr.pitch + corr.yaw - corr.roll;
	//	left front:
	PWM_M4 = Throttle - corr.pitch - corr.yaw - corr.roll;
}

static uint16_t micros(){
	static uint16_t t1;
	uint16_t t2 = TIM2->CNT;
	if(t2 > t1)
		return t2 - t1;
	return (uint16_t)TIM2->ARR + 1 + t2 - t1;
}

static void gyro_angles(ThreeD *gyro_angles){
	gyro_angles->roll 	+=	 (Gyro_Acc[0] + GYRO_ROLL_OFFSET) * dt;
	gyro_angles->pitch 	+=	 (Gyro_Acc[1] + GYRO_PITCH_OFFSET) * dt;
	gyro_angles->yaw 	+=	 (Gyro_Acc[2] + GYRO_YAW_OFFSET) * dt;
}

static void acc_angles(){
	static Three acc_outcome[MEDIAN_BUFFOR];
	for(int i = MEDIAN_BUFFOR - 1; i > 0; i--)
		acc_outcome[i] = acc_outcome[i-1];
	acc_outcome[0].roll 	= 	Gyro_Acc[3];
	acc_outcome[0].pitch 	= 	Gyro_Acc[4];
	acc_outcome[0].yaw 		= 	Gyro_Acc[5];
	Three acc_filtered = median_filter(acc_outcome);
	acc_angle_roll 	= 	atan2(acc_filtered.pitch, acc_filtered.yaw) * rad_to_deg;
	acc_angle_pitch	=	atan2(-acc_filtered.roll, sqrt(acc_filtered.pitch*acc_filtered.pitch
		+ acc_filtered.yaw*acc_filtered.yaw));
}

static void complementary_filter(){
	gyro_angles(&angles);
	acc_angles();
	angles.roll		=	ACC_PART * acc_angle_roll + GYRO_PART * angles.roll;
	angles.pitch	=	ACC_PART * acc_angle_pitch + GYRO_PART * angles.pitch;
}

static Three median_filter(Three values[]){
	Three filtered_values = values[0];
	Three count;
	for(int i = 0; i < MEDIAN_BUFFOR; i++){
		count.roll = 0;
		count.pitch = 0;
		count.yaw = 0;
		for(int j = 0; j < i; j++){
			if(values[i].roll <= values[j].roll)
				count.roll += 1;
			if(values[i].pitch <= values[j].pitch)
				count.roll += 1;
			if(values[i].pitch <= values[j].pitch)
				count.roll += 1;
			}
		for(int j = i+1; j < MEDIAN_BUFFOR; j++){
			if(values[i].roll <= values[j].roll)
				count.roll += 1;
			if(values[i].pitch <= values[j].pitch)
				count.roll += 1;
			if(values[i].pitch <= values[j].pitch)
				count.roll += 1;
			}
		if(count.roll == MEDIAN_BUFFOR / 2)
			filtered_values.roll = values[i].roll;
		if(count.pitch == MEDIAN_BUFFOR / 2)
			filtered_values.pitch = values[i].pitch;
		if(count.yaw == MEDIAN_BUFFOR / 2)
			filtered_values.yaw = values[i].yaw;
		}

	return filtered_values;
}

static Three rates_PID(){
	// err values - difference between set value and measured value:
	Three err;
	Three corr;
	static Three last_err 	=	{0, 0, 0};
	static Three sum_err	=	{0, 0, 0};
	PID R_PID 	=	 {1,1,1};
	PID P_PID 	=	 {1,1,1};
	PID Y_PID 	=	 {1,1,1};
	err.roll	=	channels[0] - angles.roll;
	err.pitch	=	channels[1] - angles.pitch;
	err.yaw		=	channels[2] - angles.yaw;

		//	estimate Integral by sum (I term):
	sum_err.roll 	+=	 err.roll;
	sum_err.pitch 	+=	 err.pitch;
	sum_err.yaw		+=	 err.yaw;

		//	calculate corrections:
	corr.roll	=	R_PID.P * err.roll + R_PID.I*sum_err.roll + R_PID.D * (err.roll - last_err.roll) / dt;
	corr.pitch	=	P_PID.P * err.pitch + P_PID.I*sum_err.pitch + P_PID.D * (err.pitch - last_err.pitch) / dt;
	corr.yaw	=	Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw + Y_PID.D * (err.yaw - last_err.yaw) / dt;

		//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;
	return corr;
}

static Three angles_PID(Three rates){
	// err values - difference between set value and measured value:
	Three err;
	Three corr;
	static Three last_err 	=	{0, 0, 0};
	static Three sum_err	=	{0, 0, 0};
	PID R_PID 	=	 {1,1,1};
	PID P_PID 	=	 {1,1,1};
	PID Y_PID 	=	 {1,1,1};
	err.roll	=	Gyro_Acc[0] - rates.roll * 32768/1000;
	err.pitch	=	Gyro_Acc[1] - rates.pitch * 32768/1000;
	err.yaw		=	Gyro_Acc[2] - rates.yaw * 32768/1000;

		//	estimate Integral by sum (I term):
	sum_err.roll 	+=	 err.roll;
	sum_err.pitch 	+=	 err.pitch;
	sum_err.yaw		+=	 err.yaw;

		//	calculate corrections:
	corr.roll	=	(R_PID.P * err.roll + R_PID.I*sum_err.roll + R_PID.D * (err.roll - last_err.roll) / dt)*1000/32768;
	corr.pitch	=	(P_PID.P * err.pitch + P_PID.I*sum_err.pitch + P_PID.D * (err.pitch - last_err.pitch) / dt)*1000/32768;
	corr.yaw	=	(Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw + Y_PID.D * (err.yaw - last_err.yaw) / dt)*1000/32768;

		//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;
	return corr;
}
