/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"
#include "setup.h"
#include "MPU6050.h"
#include "acro.h"
#include "stabilize.h"
#include "ibus.h"


//	konfiguracja uzytkownika:

uint8_t CHANNELS=10;		//ilosc kanalĂłw (4 potrzebne do sterownaia)




void print(uint16_t x, int8_t nr,uint8_t enter);
void update_motors();
int failsafe();
void LED ();

// zmienne do debugowania:
float test1;
float test2;
float test3;



// global data
#define  Gyro_Acc_Size 7

uint16_t current_time=0;
uint16_t last_time=0;
uint16_t gap_time=0;
uint8_t txDone=1;
uint16_t channels[14];
int32_t gyro_X=0;
int32_t gyro_Y=0;
int32_t gyro_Z=0;
int32_t acc_X;
int32_t acc_Y;
int32_t acc_Z;
uint32_t data;
int16_t Gyro_Acc[Gyro_Acc_Size];

uint16_t motors[4];

uint16_t PWM_M1=1000;
uint16_t PWM_M2=1000;
uint16_t PWM_M3=1000;
uint16_t PWM_M4=1000;

uint16_t Throttle;
int16_t Pitch;
int16_t Roll;
int16_t Yaw;

uint8_t I2C1_read_write_flag = 1;

volatile uint8_t time[] = "0000  ";

extern int8_t new_I_Bus;
extern volatile int32_t g_txSize;
extern volatile int32_t g_txTransmitted;

int main(void)
{
	setup();
	setup_MPU6050();

	int8_t channel_nr=0;
	int8_t gyro_nr=0;
	while (1) {

	if(I2C1_read_write_flag) {
		read_all();
	}

	volatile int16_t podglad[7];
	for(int i=0; i<7; i++) podglad[i] = Gyro_Acc[i];
	static int counter = 0;
	static double srednia[3] = {0};
	for (int i = 0;i<3;i++)
		srednia[i] += podglad[i];
	counter++;
	static int kanaly[4];
	for (int i =0; i<4;i++)
		kanaly[i]=channels[i];

	Throttle=channels[2];
		// if failsafe occurs set motors to 0 rpm:

	if(failsafe()){
		PWM_M1=1000;
		PWM_M2=1000;
		PWM_M3=1000;
		PWM_M4=1000;
	}
	else if(channels[6]<1450){
		acro();
	}
	else if(channels[6]>1400){
		stabilize();
	}

	update_motors();

	Ibus_save();
	if (0!=new_I_Bus && 0!=txDone) {
			// Transmit data

			print(channels[channel_nr],channel_nr,CHANNELS);

			channel_nr++;

			if(channel_nr>=CHANNELS){
				channel_nr=0;
				new_I_Bus=0;

			}
	}
		}
	}

void LED ()
{
	PWM_M1=(PWM_M1-1000)*20000;
	PWM_M2=(PWM_M2-1000)*20000;
	PWM_M3=(PWM_M3-1000)*20000;
	PWM_M4=(PWM_M4-1000)*20000;
}
void print(uint16_t x, int8_t nr,uint8_t enter){

	time[0] = '0' + (x % 10000) / 1000; //tysiace
	time[1] = '0' + (x % 1000)/100;		//setki
	time[2] = '0' + (x % 100)/10;		//dziesi
	time[3] = '0' + (x % 10);			//jednosci
	if(nr==enter-1){
	time[4] = '\r';
	time[5] = '\n';
	}
	else{
		time[4]=' ';
		time[5]=' ';
	}

	g_txSize = 6;
	g_txTransmitted = 0;
	txDone=0;
	USART2->CR1 |= USART_CR1_TXEIE;
}






void update_motors(){
	TIM2->CCR1 =PWM_M1 - 1; 			//wypelneinie motor 1
	TIM2->CCR2 =PWM_M2 - 1; 			//wypelneinie motor 2
	TIM2->CCR3 =PWM_M3 - 1; 			//wypelneinie motor 3
	TIM2->CCR4 =PWM_M4 - 1; 			//wypelneinie motor 4

}


int failsafe(){
	// Arming switch - SA
	if(channels[4]<=1600){
	return(1);
	}
	// if failsafe not appears:
	else{
	return(0);
	}
}
