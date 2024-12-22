/*
 *  13_i2c_slave_tx_string.c
 *  Created on: 23-Jun-2024
 *  Author: Himanshu Singh
 */

#include<stdio.h>
#include<string.h>

#include "stm32f401xx.h"

#define MY_ADDR 0x68

void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

//some data
uint8_t Tx_buf[32]= "stm32 slave mode testing";

I2C_Handle_t I2C1Handle;
/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_OP_TYPE_PP;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}




void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void){
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void){
	GPIO_ButtonInit();
	//i2c pin inits
	I2C1_GPIOInits();

	// I2C PERIPHERAL configuraion
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	// ENABLE THE PERIPHERAL
	I2C_PeripheralControl(I2C1, ENABLE);

	// ACK bit is made 1 after PE =1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);
	while(1);
}

void I2C1_EV_IRQHandler (void){
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void){
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){

	static uint8_t commandCode =0;
	static uint8_t Cnt =0;
    if(AppEv == I2C_EV_DATA_REQ){
        // Master Wants Some Data, slave has to send it
    	if(commandCode == 0x51){
    	    // send the length information to the master
    		I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)Tx_buf));
    	}
    	else if(commandCode == 0x52){
    		 // send the contents of Tx_buf
    		 I2C_SlaveSendData(pI2CHandle->pI2Cx,Tx_buf[Cnt++]);
    	}

    } else if (AppEv == I2C_EV_DATA_RCV){
    	// data is waiting for slave to read. slave has to read it
    	commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

    } else if (AppEv == I2C_ERROR_AF){
    	// this happens only during slave Txing
    	// master has sent NACK. so slave should understand that master does not need more data
    	commandCode = 0xff;
    	Cnt = 0;

    } else if (AppEv == I2C_EV_STOP){
    	 // THIS happens during slave reception
         // master has ended the i2c communication with slave.
    }
}


