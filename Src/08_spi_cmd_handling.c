/*
 * 08_spi_cmd_handling.c
 *
 *  Created on: 31-May-2024
 *      Author: Himanshu Singh
 */

#include<string.h>
#include"stm32f401xx.h"




// command codes
#define COMMAND_LED_CTRL         0x50
#define COMMAND_SENSOR_READ      0x51
#define COMMAND_LED_READ         0x52
#define COMMAND_PRINT            0x53
#define COMMAND_ID_READ          0x54

#define LED_ON   1
#define LED_OFF  0

// arduino Analog pins
#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4

// arduino led
#define LED_PIN  9

/*
 *  PB14 --> SPI2_MISO
 *  PB15 --> SPI2_MOSI
 *  PB13 --> SPI2_SCL
 *  PB12 --> SPI2_NSS
 *  ALT function mode: 5
 */
void SPI2_GPIOInits() {
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

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}
void SPI2_SPI_Inits() {
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
}
;

void GPIO_ButtonInit(void) {
	GPIO_Handle_t Gpiobtn;
	Gpiobtn.pGPIOx = GPIOA;

	Gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpiobtn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_NO_PUPD;

	GPIO_Init(&Gpiobtn);
}

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {
	if (ackbyte == 0xF5) {
		// ACK
		return 1;
	}
	return 0;
}
int main(void) {
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;


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
	while (1) {
		// wait till the button is presed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid the de bouncing related issues 200ms of delay
		delay();

		// enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>   <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the rrsponse from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send arguments
			SPI_SendData(SPI2, args, 2);
		}
		// end of command_led_ctrl

		// 2, CMD_SENSOR_READ  <analog pin number (1)>
		// wait till the button is presed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid the de bouncing related issues 200ms of delay
		delay();
		// send command
		commandcode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the rrsponse from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = ANALOG_PIN0;
			// send arguments
			SPI_SendData(SPI2,args,1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay so that slaves can ready with the data
			delay();
			// send some dummy bits (1byte) to fetch the response from the slave.
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		// LETS CONFRIM SPI IS NOT BUSY
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable the spi2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}
