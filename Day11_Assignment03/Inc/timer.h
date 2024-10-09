#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f4xx.h"

#define TCLK	16000000UL
#define PR		16000
extern volatile int count;
void Timer_Init(uint32_t ms);
void TIM6_DAC_IRQHandler(void);
#endif /* TIMER_H_ */
