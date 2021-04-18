/*
 * MPU6050.h
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */

#ifndef MPU6050_H_
#define MPU6050_H_

// MUST BE DIVISIBLE BY 4  How many samples for median filter (average is computed from a half of samples):
#define MEDIAN_BUFFOR 16

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




 extern int16_t median_values[][MEDIAN_BUFFOR];

void setup_MPU6050();
void I2C_Start(uint16_t);
void I2C_StartWrite(uint16_t);
void I2C_StartRead(uint16_t);

void gyro_read();
void acc_read();
void tem_read();
void read_all();


#endif /* MPU6050_H_ */
