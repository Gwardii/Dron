/*
 * global_variables.h
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

extern uint16_t channels[];

extern int16_t Throttle;

extern uint16_t pwm_m1;
extern uint16_t pwm_m2;
extern uint16_t pwm_m3;
extern uint16_t pwm_m4;

extern uint16_t *PWM_M1;
extern uint16_t *PWM_M2;
extern uint16_t *PWM_M3;
extern uint16_t *PWM_M4;

extern int16_t Gyro_Acc[];

extern uint16_t table_to_send[];

extern uint8_t New_data_to_send;

extern uint8_t ibus_received;

extern uint8_t I2C1_read_write_flag;

extern uint8_t transmitting_is_Done;

extern uint8_t failsafe_type;

extern uint16_t motor_off;

#endif /* GLOBAL_VARIABLES_H_ */
