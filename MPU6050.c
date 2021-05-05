/*
 * MPU6050.c
 *
 *  Created on: 03.01.2021
 *
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "MPU6050.h"

#include <math.h>

static void setup_conf();
static void setup_gyro();
static void setup_acc();
static void read(uint8_t, uint8_t);
void failsafe_CONF();
void failsafe_I2C();

double M_rotacji[3][3] = { { (ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_X_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_X_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_X_X - ACC_PITCH_OFFSET,
										2)) }, { (ACC_CALIBRATION_Y_X
		- ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_Y_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Y_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Y_X - ACC_PITCH_OFFSET,
										2)) }, { (ACC_CALIBRATION_Z_X
		- ACC_PITCH_OFFSET)
		/ sqrt(
				pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
						+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET, 2)),
		(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
										2)), (ACC_CALIBRATION_Z_Z
				- ACC_YAW_OFFSET)
				/ sqrt(
						pow(ACC_CALIBRATION_Z_Z - ACC_YAW_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_Y - ACC_ROLL_OFFSET, 2)
								+ pow(ACC_CALIBRATION_Z_X - ACC_PITCH_OFFSET,
										2)) } };

uint8_t read_write_tab[14];
static volatile uint8_t read_write_quantity;
static double time_flag4_1;

//for debugging only:
static uint32_t pak1 = 0;

void I2C1_IRQHandler() {
	static uint8_t i = 0;
	if (I2C1->ISR |= I2C_ISR_RXNE) {
		read_write_tab[i] = I2C1->RXDR;
		i++;
		if (i > read_write_quantity - 1) {
			i = 0;

			I2C1_read_write_flag = 1;

		}
	}
}

void EXTI4_15_IRQHandler() {
	if ((EXTI->PR & EXTI_PR_PIF4)) {
		EXTI->PR |= EXTI_PR_PIF4; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF5)) {
		EXTI->PR |= EXTI_PR_PIF5; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF6)) {
		EXTI->PR |= EXTI_PR_PIF6; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF7)) {
		EXTI->PR |= EXTI_PR_PIF7; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF8)) {
		EXTI->PR |= EXTI_PR_PIF8; // clear this bit setting it as
	}
	//Interrupt from IMU:
	if ((EXTI->PR & EXTI_PR_PIF9)) {

		EXTI->PR |= EXTI_PR_PIF9; // clear this bit setting it as 1

		if (I2C1_read_write_flag == 1) {
			EXTI->IMR &= ~EXTI_IMR_IM9;
			USART2->CR1 &= ~USART_CR1_RXNEIE;
			//	USART2->CR1 &= ~USART_CR1_IDLEIE;

			//for debugging only:
			for (int i = 0; i < 14; i++) {
				read_write_tab[i] = 0;
			}

			read_all();
			pak1++;
		}
//		if (I2C1_read_write_flag != 1){
//
//			I2C1->CR1&=~I2C_CR1_PE;
//			DMA1_Channel3->CCR &= ~DMA_CCR_EN;
//			if (ibus_received==0){
//			USART2->CR1 |= USART_CR1_RXNEIE;
//			USART2->CR1 |= USART_CR1_IDLEIE;
//			}
//			while (I2C1->CR1 & I2C_CR1_PE){
//
//			}
//			I2C1->CR1|=I2C_CR1_PE;
//
//			I2C1_read_write_flag = 1;
//		}
	}
	if ((EXTI->PR & EXTI_PR_PIF10)) {
		EXTI->PR |= EXTI_PR_PIF10; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF11)) {
		EXTI->PR |= EXTI_PR_PIF11; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF12)) {
		EXTI->PR |= EXTI_PR_PIF12; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF13)) {
		EXTI->PR |= EXTI_PR_PIF13; // clear this bit setting it as
	}
	if ((EXTI->PR & EXTI_PR_PIF14)) {
		EXTI->PR |= EXTI_PR_PIF14; // clear this bit setting it as
	}
	//FAILSAFE ARM SWITCH is set as DISARM:
	if ((EXTI->PR & EXTI_PR_PIF15)) {
		EXTI->PR |= EXTI_PR_PIF15; // clear(setting 1) this bit (and at the same time bit SWIER15)
		TIM2->CCR1 = 1000 - 1; 			//wypelneinie motor 1
		TIM2->CCR2 = 1000 - 1; 			//wypelneinie motor 2
		TIM2->CCR3 = 1000 - 1;			//wypelneinie motor 3
		TIM2->CCR4 = 1000 - 1; 			//wypelneinie motor 4
		PWM_M1 = &motor_off;
		PWM_M2 = &motor_off;
		PWM_M3 = &motor_off;
		PWM_M4 = &motor_off;
		switch (failsafe_type) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		default:
		}
	}
}

void setup_MPU6050() {
	setup_conf();
	setup_gyro();
	setup_acc();

}
void I2C_Start(uint16_t Number_of_Bytes) {
	// Ile bajtów bêdzie wysy³ane:
	I2C1->CR2 = ((~0xF0000 & (I2C1->CR2)) | Number_of_Bytes << 16);
	// wys³anie bajtu START aby rozpocz¹c komunikacje:
	I2C1->CR2 |= I2C_CR2_START;
	time_flag4_1 = get_Global_Time();
	while (I2C1->CR2 & I2C_CR2_START) {
		// czekam az START w CR2 zosatnie wyczyszczony aby wys³ac kolejne bajty
		failsafe_I2C();
	}
}
void I2C_StartWrite(uint16_t Number_of_Bytes) {
	// transfer direction 0-write 1-read:
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;
	// inicjalizacja komunikacji:
	I2C_Start(Number_of_Bytes);
}
void I2C_StartRead(uint16_t Number_of_Bytes) {
	// transfer direction 0-write 1-read
	I2C1->CR2 |= I2C_CR2_RD_WRN;
	// inicjalizacja komunikacji:
	I2C_Start(Number_of_Bytes);
}

void read_all() {
	I2C1_read_write_flag = 0;
	read(0x3B, 14);

}
static void setup_conf() {
	//-------main MPU6050 setting-----------

	//	slave address shifted by 1:
	I2C1->CR2 |= 0x68 << 1;
	//	start communication:
	I2C_StartWrite(2);
	// address of Power Management 1 register:
	I2C1->TXDR = 0x6B;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// set 0x80 in this register (RESET)
	I2C1->TXDR = 0x80;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	//delay of 0.1[s] according the MPU6050 datasheet
	delay_mili(150);

	//	slave address shifted by 1:
	I2C1->CR2 |= 0x68 << 1;
	//	start communication:
	I2C_StartWrite(2);
	// address of Power Management 1 register:
	I2C1->TXDR = 0x6B;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// set 0x0 in this register (SLEEP -> 0)
	I2C1->TXDR = 0x0;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}

	//	slave address shifted by 1:
	I2C1->CR2 |= 0x68 << 1;
	//	start communication:
	I2C_StartWrite(2);
	// address of (26) Configuration register:
	I2C1->TXDR = 0x1A;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// setting low pass filter in this register (turn off)
	I2C1->TXDR = 0x01;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}

	//	slave address shifted by 1:
	I2C1->CR2 |= 0x68 << 1;
	//	start communication:
	I2C_StartWrite(2);
	// address of (36) I2C Master Control register:
	I2C1->TXDR = 0x24;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// clock divider in this register
	I2C1->TXDR = 0xD;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}

	//	slave address shifted by 1:
	I2C1->CR2 |= 0x68 << 1;
	//	start communication:
	I2C_StartWrite(2);
	// address of (56) Interrupt Enable register:
	I2C1->TXDR = 0x38;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// setting interrupt source as Data Register
	I2C1->TXDR = 0x0001;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}

}
static void setup_gyro() {
	//---------setting Gyro--------

	//start communication:
	I2C_StartWrite(2);
	// address of Gyroscope Configuration register:
	I2C1->TXDR = 0x1C;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent, failsafe if set time passed
		failsafe_CONF();
	}
	// set +/-1000[deg/s]
	I2C1->TXDR = 0x10;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent
		failsafe_CONF();
	}
}
static void setup_acc() {
	//---------setting Accelerometer-----------

	//	start communication:
	I2C_StartWrite(2);
	//	address of Accelerometer Configuration register:
	I2C1->TXDR = 0x1B;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent
		failsafe_CONF();
	}
	// set +/-8[g]
	I2C1->TXDR = 0x10;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent
		failsafe_CONF();
	}
}
static void read(uint8_t address, uint8_t n) {
	read_write_quantity = n;
	I2C_StartWrite(1);
	//	1st address of accelerometer measurements, every next reading will increase register number by 1
	I2C1->TXDR = address;
	time_flag4_1 = get_Global_Time();
	while (!(I2C1->ISR & I2C_ISR_TXE)) {
		//	waiting as Data will be sent
		failsafe_I2C();
	}
	I2C_StartRead(n);
	// enable DMA reading:
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void rewrite_data() {
	if (I2C1_read_write_flag == 1) {
		for (int i = 0; i < 3; i++) {
			//gyro:
			Gyro_Acc[i] = read_write_tab[2 * i + 8] << 8
					| read_write_tab[2 * i + 9];
			//acc:
			Gyro_Acc[i + 3] = read_write_tab[2 * i] << 8
					| read_write_tab[2 * i + 1];

		}

		double temporary[6] = { 0 };
		for (int j = 0; j < 3; j++) {

			for (int i = 0; i < 3; i++) {
				temporary[j] += Gyro_Acc[i] * M_rotacji[i][j];
				temporary[j + 3] += Gyro_Acc[i + 3] * M_rotacji[i][j];
			}
		}
		for (int i = 0; i < 6; i++) {
			Gyro_Acc[i] = temporary[i];
		}

		//temperature:
		Gyro_Acc[6] = read_write_tab[6] << 8 | read_write_tab[7];
		EXTI->IMR |= EXTI_IMR_IM9;
	}
}

void failsafe_CONF() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= MAX_I2C_TIME) {
		failsafe_type = 4;
		EXTI->SWIER |= EXTI_SWIER_SWI15;
	}
}
void failsafe_I2C() {
	//	waiting as Data will be sent or failsafe if set time passed
	if ((get_Global_Time() - time_flag4_1) >= MAX_I2C_TIME) {
		failsafe_type = 5;
		EXTI->SWIER |= EXTI_SWIER_SWI15;
	}
}
