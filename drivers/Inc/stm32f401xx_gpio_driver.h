/*
 *  stm32f401xx_gpio_driver.h
 *  gpio peripheral specific header file
 *  Created on: 11-Feb-2024
 *  Author: Himanshu Singh
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

typedef struct{
     uint8_t GPIO_PinNumber;
     uint8_t GPIO_PinMode;    // possible values from @GPIO_PIN_MODES
     uint8_t GPIO_PinSpeed;   // possible values from @GPIO_PIN_SPEED
     uint8_t GPIO_PinPupdControl;
     uint8_t GPIO_PinOPType;
     uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

// this is a handle structure or a GPIO pin
typedef struct{
	// pointer to hold the base address of gpio peripherals
	GPIO_RegDef_t *pGPIOx; // this holds the base address of Gpio port to which the pin belongs
     GPIO_PinConfig_t GPIO_PinConfig;  // this hold the gpio pin configuration setting
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 *  GPIO pin possible modes
 */
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_RFT   6

/*
 *  GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

/*  @GPIO_PIN_SPEED
 *  GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW     0
#define GPIO_SPEED_MEDIUM  1
#define GPIO_SPEED_FAST    2
#define GPIO_SPEED_HIGH    3

/*
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD      0
#define GPIO_PU           1
#define GPIO_PD           2

/*
 *  @GPIO_PIN_NUMBERS
 *  GPIO PIN NUMBERS
 */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

/*
 * returns port code for given GPIOz base address
 */
#define GPIO_BASEADDR_TO_CODE(x)  ((x==GPIOA)?0:\
		                           (x==GPIOB)?1:\
		                           (x==GPIOC)?2:\
		                           (x==GPIOD)?3:\
		                           (x==GPIOE)?4:\
		                           (x==GPIOH)?7:0)


/*
 * API Supported by this driver
 */

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

// init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Date Read And Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);    // this will return the content of input data register
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

// IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
