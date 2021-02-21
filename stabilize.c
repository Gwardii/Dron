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

#define GYRO_PART 0.995
#define ACC_PART .005
#define GYRO_TO_DPS 32768/1000. // convert gyro register into degrees per second unit

#define GYRO_ROLL_OFFSET -28.787424166100877
#define GYRO_PITCH_OFFSET 23.64611577126087
#define GYRO_YAW_OFFSET -45.658468209991462

#define MAX_ROLL_ANGLE 20
#define MAX_PITCH_ANGLE 20
#define MAX_YAW_ANGLE 180

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

static const double rad_to_deg = 180 / atan(1) / 4;
static double acc_angle_roll;
static double acc_angle_pitch;

static double dt;

static double micros();
static double milis();
static void gyro_angles(ThreeD*);
static void acc_angles();
static void complementary_filter();
static ThreeD median_filter(ThreeD *);
static ThreeD angles_PID();
static void set_motors(ThreeD);
static double anti_windup(double);

void stabilize(){
	static double time;
	time += milis();
	if(time < 20)
		return;
	time = 0;
	dt = micros();
	complementary_filter();
	set_motors(angles_PID());
}

static double micros(){
	static uint16_t t1;
	double temp;
	uint16_t t2 = TIM2->CNT;
	if(t2 > t1){
		temp = (t2 - t1)/1000000.;
	}
	else
		temp = (TIM2->ARR + 1 + t2 - t1)/1000000.;
	t1=t2;
	return temp;
}
static double milis() {
	static uint16_t t1;
	double temp;
	uint16_t t2 = TIM2->CNT;
	if (t2 > t1) {
		temp = (t2 - t1) / 100.;
	} else
		temp = (TIM2->ARR + 1 + t2 - t1) / 100.;
	t1 = t2;
	return temp;
}

static void gyro_angles(ThreeD *gyro_angles){
	gyro_angles->roll 	+=	 (Gyro_Acc[0] - GYRO_ROLL_OFFSET) * dt/GYRO_TO_DPS;
	gyro_angles->pitch 	+=	 (Gyro_Acc[1] - GYRO_PITCH_OFFSET) * dt/GYRO_TO_DPS;
	gyro_angles->yaw 	+=	 (Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt/GYRO_TO_DPS;
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
	acc_angle_roll 	= 	atan2(acc_filtered.roll, acc_filtered.yaw) * rad_to_deg;
	acc_angle_pitch	=	atan2(acc_filtered.pitch, acc_filtered.yaw) * rad_to_deg;
		//atan2(-acc_filtered.roll, sqrt(acc_filtered.pitch*acc_filtered.pitch	+ acc_filtered.yaw*acc_filtered.yaw));
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
	// err values - difference between set value and measured value:
	ThreeD err;
	ThreeD corr;
	static ThreeD last_err 	=	{0, 0, 0};
	static ThreeD sum_err	=	{0, 0, 0};
	PID R_PID 	=	 {5,.2,0.1};
	PID P_PID 	=	 {5,.2,0.1};
	PID Y_PID 	=	 {.65,0.3,0};
	err.roll	=	-((channels[0] - 1500) * MAX_ROLL_ANGLE/500. - angles.roll);
	err.pitch	=	-((channels[1] - 1500) * MAX_PITCH_ANGLE/500. - angles.pitch);
	err.yaw		=	(channels[3] - 1500) - Gyro_Acc[2] * 1000 / 500 *1000/32768.;

		//	estimate Integral by sum (I term):
	if (sum_err.roll < 1500)
		sum_err.roll += err.roll;
	if (sum_err.pitch < 1500)
		sum_err.pitch += err.pitch;
	if (sum_err.yaw < 1500)
		sum_err.yaw += err.yaw;

		//	calculate corrections:
	corr.roll	=	R_PID.P * err.roll + anti_windup(R_PID.I*sum_err.roll) + anti_windup(R_PID.D * (err.roll - last_err.roll) / dt);
	corr.pitch	=	P_PID.P * err.pitch + anti_windup(P_PID.I*sum_err.pitch) + anti_windup(P_PID.D * (err.pitch - last_err.pitch) / dt);
	corr.yaw	=	Y_PID.P * err.yaw + anti_windup(Y_PID.I*sum_err.yaw) + anti_windup(Y_PID.D * (err.yaw - last_err.yaw) / dt);

		//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;
	return corr;
}

static void set_motors(ThreeD corr){
	//	Make corrections:

		//	right front:
		PWM_M1 = Throttle - corr.pitch + corr.yaw + corr.roll;
		//	right back:
		PWM_M2 = Throttle + corr.pitch - corr.yaw + corr.roll;
		//	left back:
		PWM_M3 = Throttle + corr.pitch + corr.yaw - corr.roll;
		//	left front:
		PWM_M4 = Throttle - corr.pitch - corr.yaw - corr.roll;
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

static double anti_windup(double correction){
	if(correction>300)
		return 300;
	else if(correction <-300)
		return -300;
	return correction;
}
