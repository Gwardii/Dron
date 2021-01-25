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
*/
