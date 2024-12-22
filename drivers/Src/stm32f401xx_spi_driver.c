/*
 *  stm32f401xx_spi_driver.c
 *  Created on: 20-Mar-2024
 *  Author: Himanshu Singh
 */

#include "stm32f401xx_spi_driver.h"

// these are private helper function, here static is used to make these function private, if application tries them to call then compiler will issue an error
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


// Peripheral clock setup
/*
 * @fn      -SPI_PeriClockControl
 * @brief   -This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in] - base address of gpio peripheral
 * @param[in] - ENABLE OR DISABLE macros
 * @param[in] -
 *
 * @return      - none
 *
 * @note        - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pSPIx==SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3){
			SPI3_PCLK_EN();
				}
		else if(pSPIx==SPI4){
			SPI4_PCLK_EN();
				}
	}
	else{
		if(pSPIx==SPI1){
			SPI1_PCLK_DI();
			}
		else if(pSPIx==SPI2){
			SPI2_PCLK_DI();
			}
		else if(pSPIx==SPI3){
			SPI3_PCLK_DI();
			}
		else if(pSPIx==SPI4){
			SPI4_PCLK_DI();
			}
	}
}




void SPI_Init(SPI_Handle_t *pSPIHandle){
	// enable the clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	// first lets configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode <<SPI_CR1_MSTR; // 0 or 1 will be shifted to 2nd bit position

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode should be cleared
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		// RX only bit must be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed(baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 4. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL ;

	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}




/*
 * @fn      -SPI_SendData
 * @brief   -
 *
 * @param[in] -
 * @param[in] -
 * @param[in] -
 *
 * @return      -
 *
 * @note        - this is a blocking call
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t* pTxBuffer,uint32_t Len){
	while(Len>0){
		//1. wait until TXE is set
		// while(!(pSPIx->SR)&(1<<1)) // IF TXE is not yet set then this (pSPIx->SR)&(1<<1) whole operation retrun 1
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
        //2.check the DFF bit in CR1
		if(pSPIx->CR1 & (1<< SPI_CR1_DFF)){
			// 16 bit DFF
			//1. load the data in to the data register
			pSPIx->DR = *((uint16_t*)pTxBuffer); // value comes out to be from this is 16 bit data
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else{
			// 8 bit data format
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;

		}

	}
}



void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t* pRxBuffer,uint32_t Len){
	while(Len>0){
		//1. wait until RXNE is set
		// while(!(pSPIx->SR)&(1<<1)) // IF TXE is not yet set then this (pSPIx->SR)&(1<<1) whole operation retrun 1
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
        //2.check the DFF bit in CR1
		if(pSPIx->CR1 & (1<< SPI_CR1_DFF)){
			// 16 bit DFF
			//1. load the from DR to Rx buffer address
			*((uint16_t*)pRxBuffer)= pSPIx->DR; // value comes out to be from this is 16 bit data
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else{
			// 8 bit data format
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;

		}

	}
}



/*
 * @fn      - SPI_PeripheralControl
 * @brief   -
 *
 * @param[in] -
 * @param[in] -
 * @param[in] -
 *
 * @return      -
 *
 * @note        - ENABLE OR DISABLE SPI PERIPHERAL
 */
void  SPI_PeripheralControl(SPI_RegDef_t* pSPIx,uint8_t EnOrDi){
    if(EnOrDi== ENABLE){
    	pSPIx->CR1 |= (1<<SPI_CR1_SPE);  // setting the bit
    }
    else{
    	pSPIx->CR1 &= ~(1<<SPI_CR1_SPE); // resetting the bit
    }
}




/*
 * @fn      - SPI_SSIConfig
 * @brief   -
 *
 * @param[in] -
 * @param[in] -
 * @param[in] -
 *
 * @return      -
 *
 * @note        - ENABLE OR DISABLE SPI PERIPHERAL
 */
void SPI_SSIConfig(SPI_RegDef_t* pSPIx,uint8_t EnOrDi){
	if(EnOrDi== ENABLE){
	    	pSPIx->CR1 |= (1<<SPI_CR1_SSI);  // setting the bit
	    }
	    else{
	    	pSPIx->CR1 &= ~(1<<SPI_CR1_SSI); // resetting the bit
	    }
}


/*
 * @fn      - SPI_SSOEConfig
 * @brief   -
 *
 * @param[in] -
 * @param[in] -
 * @param[in] -
 *
 * @return      -
 *
 * @note        - ENABLE OR DISABLE SSOE
 */
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx,uint8_t EnOrDi){
	if(EnOrDi== ENABLE){
	    	pSPIx->CR2 |= (1<<SPI_CR2_SSOE);  // setting the bit
	    }
	    else{
	    	pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE); // resetting the bit
	    }
}




void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
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



void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	// 1. First lets find out the ipr register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority<<shift_amount);
}




uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t* pTxBuffer,uint32_t Len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX){
	    // 1. save the tx buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. mark the SPI state busy in transmission so that no other code take over the same SPI peripheral untill transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. enable the TXEIE control bit to get interrupt whenever TXE flag is set in Status register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}



uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t* pRxBuffer,uint32_t Len){
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX) {
		// 1. save the tx buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. mark the SPI state busy in transmission so that no other code take over the same SPI peripheral untill transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. enable the RXNEIE control bit to get interrupt whenever TXE flag is set in Status register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}



void SPI_IRQHandling(SPI_Handle_t* pHandle){
	uint8_t temp1, temp2;

	// first lets check for TXE
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXE); // IF TXE flag will set temp1 contains 1 or vice versa
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if(temp1 && temp2){
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//  check for RXNE
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXNE); // IF TXE flag will set temp1 contains 1 or vice versa
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2){
		// handle TXE
		spi_rxne_interrupt_handle(pHandle);
    }

	//  check for OVR FLAG
	temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR); // IF TXE flag will set temp1 contains 1 or vice versa
	temp2 = pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if(temp1 && temp2){
		// handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}




// some helper function implementations
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	// check the DFF bit in CR1
		if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if(pSPIHandle->TxLen == 0){
			// Txlen is zero , so close the transmission and inform the appplication that tx is over
			// this prevents interrupts from setting up of TXE flag
			SPI_CloseTransmisson(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;   // here temp is unuced variable so making it temp
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}




























