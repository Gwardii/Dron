/*
 * kosz.c
 *
 *  Created on: 25.01.2021
 *      Author: filip
 */

//przerwanie czytajace PWM nie istotne ale teoretycznie mozna go uzywac zamiast ibusa (IBUS lepszy)
/*
void TIM2_IRQHandler(void) 						// wywolywane kiedy wykryta jest zmiana syganlu
{
	if (0 != (TIM_SR_CC2IF & TIM2->SR)) { 		// check "Capture/compare 1 interrupt flag"
	TIM2->SR &= ~TIM_SR_CC2IF; 					// clear interrupt flag
	time_obecny=TIM2->CNT;
	 if(!((TIM2->CCER) & TIM_CCER_CC2P)) { 		//wznoszacy brzeg
		 TIM2->CCER |=TIM_CCER_CC2P;				//zmiana wywolania przerwania wznoszacy na opadajacy   brzeg
	 }
	 else if(((TIM2->CCER) & TIM_CCER_CC2P)){ 			//opadajacy brzeg
		 TIM2->CCER &=~TIM_CCER_CC2P;			//zmiana wywolania przerwania opadajacy na wznoszacy  brzeg
	 }
	 // sprowadzenie do wartosci dodatnich:
		if (time_obecny>time_poprzedni){
			time_counter=time_obecny-time_poprzedni;
		}
		else{
			time_counter=time_poprzedni-time_obecny;
		}
 	 time_poprzedni=time_obecny;
	 dataFlag=1;
	}
}

static Three angles_PID(Three rates){
	// err values - difference between set value and measured value:
	Three err;
	Three corr;
	static Three last_err 	=	{0, 0, 0};
	static Three sum_err	=	{0, 0, 0};
	PID R_PID 	=	 {1,1,1};
	PID P_PID 	=	 {1,1,1};
	PID Y_PID 	=	 {1,1,1};
	err.roll	=	Gyro_Acc[0] - rates.roll * 32768/1000;
	err.pitch	=	Gyro_Acc[1] - rates.pitch * 32768/1000;
	err.yaw		=	Gyro_Acc[2] - rates.yaw * 32768/1000;

		//	estimate Integral by sum (I term):
	sum_err.roll 	+=	 err.roll;
	sum_err.pitch 	+=	 err.pitch;
	sum_err.yaw	+=	 err.yaw;

		//	calculate corrections:
	corr.roll	=	(R_PID.P * err.roll + R_PID.I*sum_err.roll + R_PID.D * (err.roll - last_err.roll) / dt)*1000/32768;
	corr.pitch	=	(P_PID.P * err.pitch + P_PID.I*sum_err.pitch + P_PID.D * (err.pitch - last_err.pitch) / dt)*1000/32768;
	corr.yaw	=	(Y_PID.P * err.yaw + Y_PID.I*sum_err.yaw + Y_PID.D * (err.yaw - last_err.yaw) / dt)*1000/32768;

		//	set current errors as last errors:
	last_err.roll	=	err.roll;
	last_err.pitch	=	err.pitch;
	last_err.yaw	=	err.yaw;
	return corr;
}
*/
