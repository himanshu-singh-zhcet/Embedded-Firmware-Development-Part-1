/*
 *  05_button_interrupt.c
 *  Created on: 14-Mar-2024
 *  Author: Himanshu Singh
 */

#include<string.h>

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BUTTON_PRESSED LOW

void delay(void){
	// this will introduce 200ms delay when system clock is 16MHz
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){
		GPIO_Handle_t GpioLed,Gpiobtn;
		memset(&GpioLed,0,sizeof(GpioLed));
		memset(&Gpiobtn,0,sizeof(Gpiobtn));
		// this us led gpio configuration
		GpioLed.pGPIOx = GPIOD;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
		GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
		GpioLed.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_NO_PUPD;
		GPIO_PeriClockControl(GPIOD, ENABLE);
		GPIO_Init(&GpioLed);

		// this is button gpio configuration
		Gpiobtn.pGPIOx = GPIOD;
		Gpiobtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
		Gpiobtn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IT_FT;
		Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
		Gpiobtn.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_PU;   //using internal pull up
		GPIO_PeriClockControl(GPIOD, ENABLE);
		GPIO_Init(&Gpiobtn);

		// IRQ configurations
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI5_9,NVIC_IRQ_PRI15);
        GPIO_IRQInterruptConfig(IRQ_NO_EXTI5_9,ENABLE);
        while(1);
	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay(); // 200ms
	GPIO_IRQHandling(GPIO_PIN_NO_5); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
