/*
 *  07_spi_txonly_arduino.c
 *  Created on: 24-May-2024
 *  Author: Himanshu Singh
 */

#include<string.h>
#include"stm32f401xx.h"

/*
 *  PB14 --> SPI2_MISO
 *  PB15 --> SPI2_MOSI
 *  PB13 --> SPI2_SCL
 *  PB12 --> SPI2_NSS
 *  ALT function mode: 5
 */

void SPI2_GPIOInits(){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
//
//	// MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_SPI_Inits(){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generates serial clock of 2 MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management for NSS Pin
    SPI_Init(&SPI2handle);
};

void GPIO_ButtonInit(void){
	GPIO_Handle_t Gpiobtn;
	Gpiobtn.pGPIOx = GPIOA;

	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IN;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPupdControl=GPIO_NO_PUPD;

	GPIO_Init(&Gpiobtn);
}

void delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int main(void){
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	SPI2_GPIOInits(); // this function is used to intialize the GPIO pins to behave as SPI2 pins
	SPI2_SPI_Inits(); // this function is used to initalize SPI2 Peripheral Parameters

	/*
     * making ssoe 1 does tghe NSSoutput enable
     * the NSSpin is automatically managed by the hardware
     * i.e when spe = 1 , nss will be pulled to low
     * and nss pin will be high when spe = 0
     */
    SPI_SSOEConfig(SPI2, ENABLE);
    while(1){
	// wait till the button is presed
    while(! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

    // to avoid the de bouncing related issues 200ms of delay
    delay();

	// enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	// FIRST SEND LENGTH INFORMATION I
	uint8_t datalen = strlen(user_data);
	SPI_SendData(SPI2,&datalen,1);

	// tO send data
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

	// LETS CONFRIM SPI IS NOT BUSY
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	// disable the spi2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);
    }
	return 0;
}
