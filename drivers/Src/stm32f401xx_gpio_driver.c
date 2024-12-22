/*
 *  stm32f401xx_gpio_driver.c
 *  Created on: 11-Feb-2024
 *  Author: Himanshu Singh
 */

#include "stm32f401xx_gpio_driver.h"
// Peripheral clock setup
/*
 * @fn      -GPIO_PeriClockControl
 * @brief   -This function enables or disables peripheral clock for the given gpio port
 *
 * @param[in] - base address of gpio peripheral
 * @param[in] - ENABLE OR DISABLE macros
 * @param[in] -
 *
 * @return      - none
 *
 * @note        - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
				}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
				}
		else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
				}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
				}
	}
	else{
		if(pGPIOx==GPIOA){
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx==GPIOB){
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx==GPIOC){
					GPIOC_PCLK_DI();
						}
				else if(pGPIOx==GPIOD){
					GPIOD_PCLK_DI();
						}
				else if(pGPIOx==GPIOE){
					GPIOE_PCLK_DI();
						}
				else if(pGPIOx==GPIOH){
					GPIOH_PCLK_DI();
						}
	}
}
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	// enable the peripheral clock
	uint32_t temp =0; // temp register
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	// 1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |=temp;   // setting
	}
	else{
		// this part will code later. (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
			// 1. Configure the FTSR
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
			// 1. Configure the RTSR
			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){
			// 1. Configure both FISR and RTSR
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]= portcode<<(temp2*4);
		// 3. enable the exti interrupt delivery with IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp =0;
	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;
	temp =0;
	// 3. configure the pull up pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPupdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=temp;
	temp = 0;
	// 4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPupdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;
	temp = 0;
	// 5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		// then configure the alternate function registers
		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8; // dividing pin number by 8
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	        if(pGPIOx==GPIOA){
				GPIOA_REG_RESET();
			}
			else if(pGPIOx==GPIOB){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx==GPIOC){
				GPIOC_REG_RESET();
					}
			else if(pGPIOx==GPIOD){
				GPIOD_REG_RESET();
					}
			else if(pGPIOx==GPIOE){
				GPIOE_REG_RESET();
					}
			else if(pGPIOx==GPIOH){
				GPIOH_REG_RESET();
					}
}

/*
 * @fn      -GPIO_ReadFromInputPin
 * @brief   -
 *
 * @param[in] -
 * @param[in] -
 * @param[in] -
 *
 * @return      - 0 or 1
 *
 * @note        - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if(Value == GPIO_PIN_SET){
		// write 1 to output data register at the bit field corresponding pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		// write 0 to output data register at the bit field corresponding pin number
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);    // xor configuration
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(IRQNumber<=31){
			// Program ISER0 Register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber<64){ // 32 TO 63
			// program ISER1 Register
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=96 && IRQNumber<96){
			// program ISER2 Register
			*NVIC_ISER2 |= (1<<(IRQNumber%64));
		}
	}
	else{
		if(IRQNumber<=31){
			// Program ICER0 Register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber<64){ // 32 TO 63
			// program ICER1 Register
			*NVIC_ICER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber>=96 && IRQNumber<96){
			// program ICER2 Register
			*NVIC_ICER2 |= (1<<(IRQNumber%64));
		}
	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	// 1. First lets find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority<<shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber){
// clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber)){
		// CLEAR THAT PENDING REGISTER BIT
		EXTI->PR |= (1<<PinNumber);
	}
}
