/*
 *  01_led_toggle.c
 *  Created on: 08-Mar-2024
 *  Author: Himanshu Singh
 */

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

void delay(void){
	for(uint32_t i=0;i<500000;i++);
}

int main(void){
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	// GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_OD; // for open drain configuration
	// GpioLed.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_PU;    // FOR OPEN DRAIN CONFIGURATION AND IN INTERNAL PULL UP RESISITOR INTENSITY OF LED WILL BE VERY LOW
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
}


