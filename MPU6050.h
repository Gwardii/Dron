/*
 * MPU6050.h
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */

#ifndef MPU6050_H_
#define MPU6050_H_

void setup_MPU6050();

void I2C_Start(uint16_t);
void I2C_StartWrite(uint16_t);
void I2C_StartRead(uint16_t);

void gyro_read();
void acc_read();
void tem_read();
void read_all();

#endif /* MPU6050_H_ */
