///*
// * calibration.c
// *
// *  Created on: 11.01.2021
// *      Author: symon
// */
//
//#include "stm32l0xx.h"
//#include "stm32l0xx_nucleo.h"
//#include "MPU6050.h"
//
//extern typedef struct{
//	int32_t roll;
//	int32_t pitch;
//	int32_t yaw;
//}Three;
//
//Three gyro_calibration();
//Three gyro_calibration();
//
//Three GYRO_OFFSET;
//
//extern int16_t Gyro_Acc[];
//
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "calibration.h"
#include "MPU6050.h"

extern int16_t Gyro_Acc[];

int16_t gyro_tab_x[1000];
int16_t gyro_tab_y[1000];
int16_t gyro_tab_z[1000];

int16_t acc_tab_x[1000];
int16_t acc_tab_y[1000];
int16_t acc_tab_z[1000];

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}ThreeXYZ;

ThreeXYZ gyro_calibration();
ThreeXYZ acc_calibration();

ThreeXYZ gyro_calibration(){
ThreeXYZ G_calibration={0,0,0};
	for(int i=0;i<1000;i++){
		gyro_read();
		for(int j=i;j>0;j--){
			if(Gyro_Acc[0]>gyro_tab_x[j-1]){
				gyro_tab_x[j]=Gyro_Acc[0];
			}
			else{
				gyro_tab_x[j]=gyro_tab_x[j-1];
			}
			if(Gyro_Acc[1]>gyro_tab_y[j-1]){
				gyro_tab_y[j]=Gyro_Acc[1];
			}
			else{
				gyro_tab_y[j]=gyro_tab_y[j-1];
			}
			if(Gyro_Acc[2]>gyro_tab_z[j-1]){
				gyro_tab_z[j]=Gyro_Acc[2];
			}
			else{
				gyro_tab_z[j]=gyro_tab_z[j-1];
			}
		}
	}
	for(int i=250;i<750;i++){
		G_calibration.x +=gyro_tab_x[i];
		G_calibration.y +=gyro_tab_y[i];
		G_calibration.z +=gyro_tab_z[i];
	}

	G_calibration.x=G_calibration.x/500;
	G_calibration.y=G_calibration.y/500;
	G_calibration.z=G_calibration.z/500;
	return G_calibration;
}

ThreeXYZ acc_calibration(){
	ThreeXYZ A_calibration={0,0,0};
	for(int i=0;i<1000;i++){
		gyro_read();
		for(int j=i;j>0;j--){
			if(Gyro_Acc[3]>gyro_tab_x[j-1]){
				gyro_tab_x[j]=Gyro_Acc[3];
			}
			else{
				gyro_tab_x[j]=gyro_tab_x[j-1];
			}
			if(Gyro_Acc[4]>gyro_tab_y[j-1]){
				gyro_tab_y[j]=Gyro_Acc[4];
			}
			else{
				gyro_tab_y[j]=gyro_tab_y[j-1];
			}
			if(Gyro_Acc[5]>gyro_tab_z[j-1]){
				gyro_tab_z[j]=Gyro_Acc[5];
			}
			else{
				gyro_tab_z[j]=gyro_tab_z[j-1];
			}
		}
	}
	for(int i=250;i<750;i++){
		A_calibration.x +=acc_tab_x[i];
		A_calibration.y +=acc_tab_y[i];
		A_calibration.z +=acc_tab_z[i];
	}

	A_calibration.x=A_calibration.x/500;
	A_calibration.y=A_calibration.y/500;
	A_calibration.z=A_calibration.z/500;

	return A_calibration;

}
