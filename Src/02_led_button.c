/*
 * 02_LED_Button.c
 *
 *  Created on: 08-Mar-2024
 *      Author: Himanshu Singh
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"
#define HIGH 1
#define LOW 0
#define BUTTON_PRESSED LOW
void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}
int main(void){
	GPIO_Handle_t GpioLed,Gpiobtn;
	// this us led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	// this is button gpio configuration
	Gpiobtn.pGPIOx = GPIOC;
	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_13;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IN;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&Gpiobtn);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)==BUTTON_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}


	}
}
