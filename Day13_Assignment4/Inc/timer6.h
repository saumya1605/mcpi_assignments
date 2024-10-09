/*
 * timer.h
 *
 *  Created on: Sep 30, 2024
 *      Author: sunbeam
 */

#ifndef TIMER6_H_
#define TIMER6_H_

#include "stm32f4xx.h"

#define TCLK	16000000UL
#define PR		16000

void Timer_Init(void);
void TIM6_DAC_IRQHandler(void);

#endif /* TIMER6_H_ */
