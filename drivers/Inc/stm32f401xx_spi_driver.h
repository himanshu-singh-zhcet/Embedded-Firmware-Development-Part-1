/*
 * stm32f401xx_spi_driver.h
 *  spi peripheral specific header file
 *  Created on: 20-Mar-2024
 *      Author: Himanshu Singh
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_
#include "stm32f401xx.h"   // INCLUDING DEVICE SPECIFIC HEADER FILE

/*
 *  Configuration structure for spix peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 *  Handle structure for spix peripheral
 */
typedef struct{
	SPI_RegDef_t    *pSPIx;        /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t    SPIConfig;
	uint8_t 	    *pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxState;	/* !< To store Tx state > */
	uint8_t 		RxState;	/* !< To store Rx state > */
}SPI_Handle_t;




/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4



/*
 *  @ SPI DEVICE Mode
 */
#define SPI_DEVICE_MODE_SLAVE  0
#define SPI_DEVICE_MODE_MASTER 1


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD               1
#define SPI_BUS_CONFIG_HD               2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3


/*
 * @SPI SclockSpeed
 */
#define SPI_SCLK_SPEED_DIV2     0
#define SPI_SCLK_SPEED_DIV4     1
#define SPI_SCLK_SPEED_DIV8     2
#define SPI_SCLK_SPEED_DIV16    3
#define SPI_SCLK_SPEED_DIV32    4
#define SPI_SCLK_SPEED_DIV64    5
#define SPI_SCLK_SPEED_DIV128   6
#define SPI_SCLK_SPEED_DIV256   7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS    0
#define SPI_DFF_16BITS   1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW     0
#define SPI_CPOL_HIGH    1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW     0
#define SPI_CPHA_HIGH    1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI       0
#define SPI_SSM_EN       1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG     (1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG    (1<<SPI_SR_RXNE)
#define SPI_BUSY_FLAG    (1<<SPI_SR_BSY)











/* *******************************************************************************
 *                            APIs supported by this driver
 *       For more information about the APIs check the function definitions
 */


// Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// init and de-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t* pTxBuffer,uint32_t Len);     // to send the data to external world, Len is size of data transfer in bites // pTxBuffer = pointer to Tx Buffer
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t* pRxBuffer,uint32_t Len);

// Data Send and Receive interrupt based API
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t* pTxBuffer,uint32_t Len);     // to send the data to external world, Len is size of data transfer in bites // pTxBuffer = pointer to Tx Buffer
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t* pRxBuffer,uint32_t Len);

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t* pHandle); // pHandle= pointer to the handle structure

// other Peripheral Control APIs
void  SPI_PeripheralControl(SPI_RegDef_t* pSPIx,uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t* pSPIx,uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t* pSPIx,uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);




/*
 *  Bit position definitions of SPI peripheral
 *  Here I am putting these in spi specific header file  but it have to put in mcu specific header file
 */

// bit position definitions for SPI_CR1
#define SPI_CR1_CPHA               0
#define SPI_CR1_CPOL               1
#define SPI_CR1_MSTR               2
#define SPI_CR1_BR                 3
#define SPI_CR1_SPE                6
#define SPI_CR1_LSBFIRST           7
#define SPI_CR1_SSI                8
#define SPI_CR1_SSM                9
#define SPI_CR1_RXONLY             10
#define SPI_CR1_DFF                11
#define SPI_CR1_CRCNEXT            12
#define SPI_CR1_CRCEN              13
#define SPI_CR1_BIDIOE             14
#define SPI_CR1_BIDIMODE           15

// bit position definitions for SPI_CR2
#define SPI_CR2_RXDMAEN            0
#define SPI_CR2_TXDMAEN            1
#define SPI_CR2_SSOE               2
#define SPI_CR2_FRF                4
#define SPI_CR2_ERRIE              5
#define SPI_CR2_RXNEIE             6
#define SPI_CR2_TXEIE             7

// bit position definitions for SPI_SR
#define SPI_SR_RXNE                0
#define SPI_SR_TXE                 1
#define SPI_SR_CHSIDE              2
#define SPI_SR_UDR                 3
#define SPI_SR_CRCERR              4
#define SPI_SR_MODF                5
#define SPI_SR_OVR                 6
#define SPI_SR_BSY                 7
#define SPI_SR_FRE                 8




#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
