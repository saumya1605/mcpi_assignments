/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "uart.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
	char str[32];
	int choice;
	UartGets(str);
	sscanf(str, '%d", &choice)');
	SystemInit();
	UartInit(9600);
	UartPuts("1.GREEN LIGHT\n2.ORANGE LIGHT\n3.RED LIGHT\n4.BLUE LIGHT");
	while(1) {
		switch(choice)
		{

		}
	}
	return 0;
}





