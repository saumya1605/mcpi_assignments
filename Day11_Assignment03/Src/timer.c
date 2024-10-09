#include "timer.h"
#include "led.h"

void Timer_Init(uint32_t ms) {
	//* Enable Timer APB clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	//* Set Timer clock -- PCLK = TCLK = 16 MHz : * Set PR = 16000
	TIM7->PSC = (PR-1);
	//* Calculate number of clocks to count (CNT).
	uint32_t cnt = (TCLK / 1000) * ms / PR;
	//* Set CNT in ARR.
	TIM7->ARR = (cnt-1);
	//* Reset Timer Counter to 0.
	TIM7->CNT = 0;
	//* Enable Timer Interrupt in DIER.
	TIM7->DIER |= TIM_DIER_UIE;
	//* Enable Timer Interrupt in NVIC.
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	//* Start Timer in CR1.
	TIM7->CR1 |= TIM_CR1_CEN;
}
volatile int count = 0;
void TIM6_DAC_IRQHandler(void) {
	//* Check if UIF flag is set (in SR).
	if(TIM7->SR & TIM_SR_UIF) {
		//* Clear UIF in SR (Ack)
		TIM7->SR &= ~TIM_SR_UIF;
		//* Perform interrupt handling -- Toggle LED
		LedToggle(LED_BLUE_PIN);
		count++;
	}
}
