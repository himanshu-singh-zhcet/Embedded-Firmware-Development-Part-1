/*
 *  00 main.c
 *  Created on: 14-Mar-2024
 *  Author: Himanshu Singh
 */
#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

int main(void){
	return 0;
}

void EXTI0_IRQHandler(void){
	// handle the interrupt
	GPIO_IRQHandling(0);
}

