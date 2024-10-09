/*
Author: Nilesh Ghule <nilesh@sunbeaminfo.com>
Course: PG-DESD @ Sunbeam Infotech
Date: Oct 1, 2024
*/

#ifndef PWM_H_
#define PWM_H_

#include "stm32f4xx.h"

#define TCLK		16000000
#define PR			16
#define ARR_Val		100

void PWM_Init(void);

#endif /* PWM_H_ */
