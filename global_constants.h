/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_


#define CHANNELS  10					//ilosc kanalĂłw (4 potrzebne do sterownaia)
#define  GYRO_ACC_SIZE 7				//3 for gyro, 3 acc and 1 for temperature
#define ALL_ELEMENTS_TO_SEND 14			//telemetry informations
#define MAX_RX_SIGNAL 2050
#define MIN_RX_SIGNAL 950
#define MAX_NO_SIGNAL_TIME 0.5 			//[s]
#define DISARM_VALUE 1600
#define MAX_I2C_TIME 0.005				//[s]
// MUST BE DIVISIBLE BY 4  How many samples for median filter (average is computed from a half of samples):
#define MEASUREMENT_BUFFOR_SIZE 8	//aktualnie nieu�ywany


// OFFSETS and CALIBRATIONS VALUE
#define GYRO_ROLL_OFFSET -28
#define GYRO_PITCH_OFFSET 25
#define GYRO_YAW_OFFSET -46

#define ACC_PITCH_OFFSET 125.6525
#define ACC_ROLL_OFFSET 2.862
#define ACC_YAW_OFFSET 391.325

#define ACC_CALIBRATION_X_X 4249.908
#define ACC_CALIBRATION_X_Y -43.456
#define ACC_CALIBRATION_X_Z 108.501

#define ACC_CALIBRATION_Y_X 184.890
#define ACC_CALIBRATION_Y_Y 4107.778
#define ACC_CALIBRATION_Y_Z 755.494

#define ACC_CALIBRATION_Z_X -114.671
#define ACC_CALIBRATION_Z_Y -279.031
#define ACC_CALIBRATION_Z_Z 4521.060

#define GYRO_PART 0.995
#define ACC_PART 0.005
#define GYRO_TO_DPS 32768/1000. // convert gyro register into degrees per second unit

#define MAX_ROLL_ANGLE 15
#define MAX_PITCH_ANGLE 15

#define FREQUENCY_PID_LOOP 250 //[Hz]
#define FREQUENCY_ESC_UPDATE 250 //[Hz]
#define FREQUENCY_IMU_READING 250 //[Hz]
#define FREQUENCY_TELEMETRY_UPDATE 50 //[Hz]


#endif /* GLOBAL_CONSTANTS_H_ */
