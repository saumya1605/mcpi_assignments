/*
 * timer.c
 *
 *  Created on: Sep 30, 2024
 *      Author: sunbeam
 */

#include <timer6.h>
#include "dac.h"
void Timer_Init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	    TIM6->PSC = 16000 - 1;

	    TIM6->ARR = 100 - 1;

	    TIM6->DIER |= TIM_DIER_UIE;

	    TIM6->CR1 |= TIM_CR1_CEN;

	    NVIC_EnableIRQ(TIM6_DAC_IRQn);

}


void TIM6_DAC_IRQHandler(void){

    if (TIM6->SR & TIM_SR_UIF) {

        TIM6->SR &= ~TIM_SR_UIF;

        static int val = 0;
        static int direction = 1;

        DAC_SetValue(val);


        val += 16 * direction;

        if (val >= 4095) {
            val = 4095;
            direction = -1;
        } else if (val <= 0) {
            val = 0;
            direction = 1;
        }
    }
}
