/*
 * stabilize.c
 *s
 */
#include <math.h>
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "MPU6050.h"
#include "stabilize.h"

#define GYRO_PART 0.975
#define ACC_PART 0.025
#define GYRO_TO_DPS 32768/1000. // convert gyro register into degrees per second unit

#define MAX_ROLL_ANGLE 15
#define MAX_PITCH_ANGLE 15

extern int16_t Gyro_Acc[];
extern int16_t channels[];
extern int16_t Throttle;
extern uint16_t PWM_M1;
extern uint16_t PWM_M2;
extern uint16_t PWM_M3;
extern uint16_t PWM_M4;
extern uint16_t table_to_send[];
extern uint8_t New_data_to_send;

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
static double acc_angle_roll=0;
static double acc_angle_pitch=0;
static double gyro_angle_roll=0;
static double gyro_angle_pitch=0;

static double dt;

static double milis();
static void gyro_angles(ThreeD*);
static void complementary_filter();
static void acc_angles();
static ThreeD angles_PID();
static void set_motors(ThreeD);
static void anti_windup();
static void median_filter(int16_t median_values[][MEDIAN_BUFFOR]);

// err values - difference between set value and measured value:
static ThreeD err={0,0,0};
static ThreeD sum_err	=	{0, 0, 0};
static ThreeD last_err 	=	{0, 0, 0};
static ThreeD D_corr={0,0,0};
static ThreeD last_D_corr={0,0,0};
static Three Rates = { 400, 400, 400 };

static PID R_PID 	=	 {0.08,0.005,0.01 };
static PID P_PID 	=	 {0.08,0.005,0.01};
static PID Y_PID 	=	 {2,0.1,0.0005};

static double srednia[6]={0};
static double counter = 0;
static double suma[6]={0};
static uint8_t loop=0;

void stabilize(){

	static double timer;
	timer += milis();
	if(timer < 10){
		return;
	}
	dt = timer/1000.;
	timer = 0;
	loop++;

	median_filter(median_values);
	//do wywalenia:
	counter++;
		for(int i=0;i<6;i++){
			suma[i] += Gyro_Acc[i];
			srednia[i] = suma[i]/counter;
		}

	// dryf zyroskopu:
	gyro_angles(&gangles);
	gyro_angle_roll=gangles.roll;
	gyro_angle_pitch=gangles.pitch;

	complementary_filter();
	set_motors(angles_PID());

	if(loop>1){
	//wypisywanie korekcji pitch P I D i roll P I D; k¹tów; zadanych wartosci
	table_to_send[0]=P_PID.P*err.pitch*500./32768.+1000;
	table_to_send[1]=P_PID.I*sum_err.pitch*500./32768.+1000;
	table_to_send[2]=P_PID.D*D_corr.pitch*500./32768.+1000;
	table_to_send[3]=R_PID.P*err.roll*500./32768.+1000;
	table_to_send[4]=R_PID.I*sum_err.roll*500./32768.+1000;
	table_to_send[5]=R_PID.D*D_corr.roll*500./32768.+1000;
	table_to_send[6]=(angles.roll/MAX_ROLL_ANGLE*50)+1000;
	table_to_send[7]=(angles.pitch/MAX_PITCH_ANGLE*50)+1000;
	table_to_send[8]=10*(gyro_angle_roll+360);
	table_to_send[9]=10*(gyro_angle_pitch+360);
	table_to_send[10]=10*(acc_angle_roll+360);
	table_to_send[11]=10*(acc_angle_pitch+360);
	table_to_send[12]=channels[1]-500;
	table_to_send[13]=channels[0]-500;

	loop=0;

	New_data_to_send=1;
	}
}

 static void acc_angles(){
acc_angle_roll 	= 	atan2(Gyro_Acc[4], Gyro_Acc[5]) * rad_to_deg+8.16;
acc_angle_pitch	=	-atan2(Gyro_Acc[3], Gyro_Acc[5]) * rad_to_deg-1;
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
	gyro_angles->roll 	+=	gyro_angles->pitch*sin((Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt/(GYRO_TO_DPS)/rad_to_deg);
	gyro_angles->pitch 	-=	 gyro_angles->roll*sin((Gyro_Acc[2] - GYRO_YAW_OFFSET) * dt/(GYRO_TO_DPS)/rad_to_deg);


}
static void complementary_filter(){
	gyro_angles(&angles);
	acc_angles();
	angles.roll			=	ACC_PART * acc_angle_roll + GYRO_PART * angles.roll;
	angles.pitch		=	ACC_PART * acc_angle_pitch + GYRO_PART * angles.pitch;
}
static ThreeD angles_PID(){
	ThreeD corr;
	static ThreeD last_channels;
	err.roll	=	((channels[0] - 1500)*32768/500. - angles.roll*32768/MAX_ROLL_ANGLE);
	err.pitch	=	((channels[1] - 1500)*32768/500. - angles.pitch*32768/MAX_PITCH_ANGLE);
	err.yaw = (channels[3] - 1500) * 32768/500. - (Gyro_Acc[2]-GYRO_YAW_OFFSET) * 1000 / Rates.yaw;

	//	estimate Integral by sum (I term):
	sum_err.roll 	+=	 err.roll*dt;
	sum_err.pitch 	+=	 err.pitch*dt;
	sum_err.yaw		+=	 err.yaw*dt;

//	//low-pass filter
//	D_corr.roll= ((err.roll-last_err.roll)/dt+last_D_corr.roll)/2.;
//	D_corr.pitch=((err.pitch-last_err.pitch)/dt+last_D_corr.pitch)/2.;
//	D_corr.yaw=((err.yaw-last_err.yaw)/dt+last_D_corr.yaw)/2.;
//
//	D_corr.roll= (err.roll-last_err.roll)/dt;
//	D_corr.pitch=(err.pitch-last_err.pitch)/dt;
//	D_corr.yaw=(err.yaw-last_err.yaw)/dt;


	D_corr.roll= -(Gyro_Acc[0]-GYRO_ROLL_OFFSET)*1000/MAX_ROLL_ANGLE+(channels[0]-last_channels.roll)/500.*32768/dt;
	D_corr.pitch=-(Gyro_Acc[1]-GYRO_PITCH_OFFSET)*1000/MAX_PITCH_ANGLE+(channels[1]-last_channels.pitch)/500.*32768/dt;
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

	last_channels.roll=channels[0];
	last_channels.pitch=channels[1];
	last_channels.yaw=channels[2];

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
static void median_filter(int16_t median_values[][MEDIAN_BUFFOR]) {
	for (uint8_t i = 0; i < 6; i++) {
		int8_t counter;
		int32_t sum=0;
		for (uint8_t j = 0; j < MEDIAN_BUFFOR; j++) {
			counter = 0;
			for (int k = 0; k < j; k++) {
				if (median_values[i][j] <= median_values[i][k]) {
					counter += 1;
				}
			}
			for (int k = j + 1; k < MEDIAN_BUFFOR; k++) {
				if (median_values[i][j] <= median_values[i][k]) {
					counter += 1;
				}
			}
			if (counter >= MEDIAN_BUFFOR / 4
					&& counter < MEDIAN_BUFFOR * 3 / 4) {
				sum += median_values[i][j];
			}
		}
		Gyro_Acc[i] = sum/(MEDIAN_BUFFOR/2);
	}
}

