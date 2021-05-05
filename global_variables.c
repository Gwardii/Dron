/*
 * global_variables.c
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"

uint16_t channels[14] = { 1500, 1500, 1000, 1500, 1000, 1000, 2000, 1000, 1000,
		1000, 1000, 1000, 1000, 1000, };

int16_t Throttle = 1000;

//PWM values set by PID's:
uint16_t pwm_m1;
uint16_t pwm_m2;
uint16_t pwm_m3;
uint16_t pwm_m4;

//pointers for PWM values:
uint16_t *PWM_M1;
uint16_t *PWM_M2;
uint16_t *PWM_M3;
uint16_t *PWM_M4;

int16_t Gyro_Acc[GYRO_ACC_SIZE]; //table for measurements 3 gyro, 1 temperature, 3 accelerometer

uint16_t table_to_send[ALL_ELEMENTS_TO_SEND];

uint8_t New_data_to_send = 0;

uint8_t ibus_received = 0;

uint8_t I2C1_read_write_flag = 1;

uint8_t transmitting_is_Done = 1;

uint8_t failsafe_type;
/* 1-disarmed 2-incorrect channels values 3-RX timeout
 * 4-setup_error 5-I2C communication error
 */

uint16_t motor_off = 1000 - 1; //value of PWM to power off motors

