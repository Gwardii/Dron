/*
 * MPU6050.c
 *
 *  Created on: 03.01.2021
 *      Author: filip
 */
#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "MPU6050.h"

static void setup_conf();
static void setup_gyro();
static void setup_acc();
static void read(uint8_t, uint8_t [], uint8_t);
static void rewrite_data();
static inline void nothing(){}

void (*after_transmission)() = nothing;

extern int16_t Gyro_Acc[];
extern int8_t dataFlag2;
extern uint8_t I2C1_read_write_flag;

uint8_t* read_write_tab;
uint8_t read_write_quantity;
uint8_t aux_tab[14];
uint8_t data_flag = 0;

void I2C1_IRQHandler(){
	static uint8_t i = 0;
	if(I2C1->ISR |= I2C_ISR_RXNE){
		read_write_tab[i] = I2C1->RXDR;
		i++;
		if(i > read_write_quantity - 1){
			i = 0;
			(*after_transmission)();
			I2C1_read_write_flag = 1;
		}
	}
}
void setup_MPU6050(){
	setup_conf();
	setup_gyro();
	setup_acc();
}

void I2C_Start(uint16_t Number_of_Bytes){
	// Ile bajtów bêdzie wysy³ane:
	I2C1->CR2 = ((~0xF0000 & (I2C1->CR2)) | Number_of_Bytes << 16);
	// wys³anie bajtu START aby rozpocz¹c komunikacje:
	I2C1->CR2 |=I2C_CR2_START;
	while(I2C1->CR2 & I2C_CR2_START){
	// czekam az START w CR2 zosatnie wyczyszczony aby wys³ac kolejne bajty
	}
}

void I2C_StartWrite(uint16_t Number_of_Bytes){
	// transfer direction 0-write 1-read:
	I2C1->CR2 &=~I2C_CR2_RD_WRN;
	// inicjalizacja komunikacji:
	I2C_Start(Number_of_Bytes);
	}

void I2C_StartRead(uint16_t Number_of_Bytes){
	// transfer direction 0-write 1-read
	I2C1->CR2 |=I2C_CR2_RD_WRN;
	// inicjalizacja komunikacji:
	I2C_Start(Number_of_Bytes);
}

void gyro_read(){

	// start communication:
	I2C_StartWrite(1);
	// 1st address of gyroscope measurements register, every next reading will increase register number by 1
	I2C1->TXDR=0x43;
	while(!(I2C1->ISR & I2C_ISR_TXE)){
	//	waiting as Data will be sent
	}

	// How many following register I want read:
	I2C_StartRead(6);

	// First and second registers reads as X:
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	// waiting as Data arrive to RXDR
	}
	// reading 1st register as upper part of 16-bits Gyro value:
	Gyro_Acc[0] = (I2C1->RXDR <<8);

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	// waiting as Data arrive to RXDR
	}
	//	reading 2nd register as lower part of 16-bits Gyro value:
	Gyro_Acc[0] |= I2C1->RXDR;

	// 3th and 4th registers reads as Y:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	Gyro_Acc[1] = (I2C1->RXDR <<8);	//zapis danych do zmiennej
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 4th register as lower part of 16-bits Gyro value:
	Gyro_Acc[1] |= I2C1->RXDR;

	// 5th and 6th registers reads as Z:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 5th register as lower part of 16-bits Gyro value:
	Gyro_Acc[2] = (I2C1->RXDR <<8);
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 6th register as lower part of 16-bits Gyro value:
	Gyro_Acc[2] |= I2C1->RXDR;

	//	setting STOP in CR2:
	I2C1->CR2 |=I2C_CR2_STOP;
	while(I2C1->CR2 & I2C_CR2_STOP){
	// 	waiting as STOP byte will be sent
	}
	//	rescale data from register to format 0-2000 (represent -1000 - 1000 [deg/s] in real)
	//Gyro_Acc[0]=Gyro_Acc[0]/32768.*1000+1000;
	//Gyro_Acc[1]=Gyro_Acc[1]/32768.*1000+1000;
	//Gyro_Acc[2]=Gyro_Acc[2]/32768.*1000+1000;
	dataFlag2=1;
}

void acc_read(){
	//	start communication:
	I2C_StartWrite(1);

	//	1st address of accelerometer measurements, every next reading will increase register number by 1
	I2C1->TXDR=0x3B;
	while(!(I2C1->ISR & I2C_ISR_TXE)){
	//	waiting as Data will be sent
	}
	// How many following register I want read:
	I2C_StartRead(6);

	// First and second registers reads as X:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 1st register as upper part of 16-bits Acc value:
	Gyro_Acc[3] = (I2C1->RXDR <<8);
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 2nd register as lower part of 16-bits Acc value:
	Gyro_Acc[3] |= I2C1->RXDR;

	// First and second registers reads as Y:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 3th register as upper part of 16-bits Acc value:
	Gyro_Acc[4] = (I2C1->RXDR <<8);
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 4th register as lower part of 16-bits Acc value:
	Gyro_Acc[4] |= I2C1->RXDR;

	// First and second registers reads as Z:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 5th register as upper part of 16-bits Acc value:
	Gyro_Acc[5] = (I2C1->RXDR <<8);
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 6th register as lower part of 16-bits Acc value:
	Gyro_Acc[5] |= I2C1->RXDR;

	//	setting STOP in CR2:
	I2C1->CR2 |=I2C_CR2_STOP;
	while(I2C1->CR2 & I2C_CR2_STOP){
	// 	waiting as STOP byte will be sent
	}

	//	rescale data from register to format 0-1600 (represent -8.00 - 8.00 [g] in real)
	//Gyro_Acc[3]=Gyro_Acc[3]*100/4096+800;
	//Gyro_Acc[4]=Gyro_Acc[4]*100/4096+800;
	//Gyro_Acc[5]=Gyro_Acc[5]*100/4096+800;

	dataFlag2=1;
}

void tem_read(){
	//start communication:
	I2C_StartWrite(1);

	//1st address of thermometer measurements, every next reading will increase register number by 1
	I2C1->TXDR=0x41;
	while(!(I2C1->ISR & I2C_ISR_TXE)){
	//	waiting as Data will be sent
	}
	//	How many following register I want read:
	I2C_StartRead(2);

	// First and second registers reads as T:

	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 1st register as upper part of 16-bits Acc value:
	Gyro_Acc[6] = (I2C1->RXDR <<8);
	while(!(I2C1->ISR & I2C_ISR_RXNE)){
	//	waiting as Data arrive to RXDR
	}
	//	reading 2nd register as lower part of 16-bits Acc value:
	Gyro_Acc[6] |= I2C1->RXDR;

	//	setting STOP in CR2:
	I2C1->CR2 |=I2C_CR2_STOP;
	while(I2C1->CR2 & I2C_CR2_STOP){
	// 	waiting as STOP byte will be sent
	}

	//	normalizing temperature value (but multiply by 100)
	Gyro_Acc[6]=Gyro_Acc[6]*100/340+3653;

	dataFlag2=1;
}
void read_all(){
	after_transmission = rewrite_data;
	I2C1_read_write_flag = 0;
	read(0x3B, aux_tab, 14);
	data_flag = 1;
}

static void setup_conf(){
	//-------main MPU6050 setting-----------

		//	slave address shifted by 1:
		I2C1->CR2 |= 0x68<<1;
		//	start communication:
		I2C_StartWrite(2);
		// address of Power Management 1 register:
		I2C1->TXDR=0x6B;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		// set 0x0 in this register (SLEEP -> 0)
		I2C1->TXDR=0x0;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		//	setting STOP in CR2:
		I2C1->CR2 |=I2C_CR2_STOP;
		while(I2C1->CR2 & I2C_CR2_STOP){
		// 	waiting as STOP byte will be sent
		}

}

static void setup_gyro(){
	//---------setting Gyro--------

	//start communication:
		I2C_StartWrite(2);
		// address of Gyroscope Configuration register:
		I2C1->TXDR=0x1C;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		// set +/-1000[deg/s]
		I2C1->TXDR=0x10;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		//	setting STOP in CR2:
		I2C1->CR2 |=I2C_CR2_STOP;
		while(I2C1->CR2 & I2C_CR2_STOP){
		// 	waiting as STOP byte will be sent
		}
}
static void setup_acc(){
	//---------setting Accelerometer-----------

		//	start communication:
		I2C_StartWrite(2);
		//	address of Accelerometer Configuration register:
		I2C1->TXDR=0x1B;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		// set +/-8[g]
		I2C1->TXDR=0x10;
		while(!(I2C1->ISR & I2C_ISR_TXE)){
		//	waiting as Data will be sent
		}
		//	setting STOP in CR2:
		I2C1->CR2 |=I2C_CR2_STOP;
		while(I2C1->CR2 & I2C_CR2_STOP){
		// 	waiting as STOP byte will be sent
		}
}
static void read(uint8_t address, uint8_t tab[], uint8_t n){
	read_write_tab = tab;
	read_write_quantity = n;
	I2C_StartWrite(1);
	//	1st address of accelerometer measurements, every next reading will increase register number by 1
	I2C1->TXDR = address;
	while(!(I2C1->ISR & I2C_ISR_TXE)){
	//	waiting as Data will be sent
	}
	// enable interrupt from RXNE flag
	I2C1->CR1 |= I2C_CR1_RXIE;
	I2C_StartRead(n);
}
static void rewrite_data(){
	for(int i =0; i<3; i++){
		Gyro_Acc[i] = read_write_tab[2*i+8] << 8 | read_write_tab[2*i+9];
		Gyro_Acc[i+3] = read_write_tab[2*i] << 8 | read_write_tab[2*i+1];
	}
	Gyro_Acc[6] = read_write_tab[6] << 8 | read_write_tab[7];
}
