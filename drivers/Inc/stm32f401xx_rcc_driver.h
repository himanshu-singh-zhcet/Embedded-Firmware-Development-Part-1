/*
 *  stm32f401xx_rcc_driver.h
 *  rcc peripheral specific header file
 *  Created on: 06-Jul-2024
 *  Author: Himanshu Singh
 */

#ifndef INC_STM32F401XX_RCC_DRIVER_H_
#define INC_STM32F401XX_RCC_DRIVER_H_

#include "stm32f401xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F401XX_RCC_DRIVER_H_ */
