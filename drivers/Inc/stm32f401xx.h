/*
 * stm32f401xx.h
 *  Device specific header file
 *  Created on: Feb 7, 2024
 *      Author: Himanshu Singh
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/************************************************************* START: Processor Specific Details *******************************************************/

/*
 * Arm Cortex M4 Processor NVIC ISERx Register address
 */
#define NVIC_ISER0 ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t*)0xE000E10C)

/*
 * Arm Cortex M4 Processor NVIC ICERx Register address
 */
#define NVIC_ICER0 ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1 ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2 ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3 ((__vo uint32_t*)0XE000E18C)

/*
 * Arm Cortex M4 Processor Priority Register address Calculation
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4 // PRIORITY BITS IS SPECIFIC TO A MICROCONTROLLER

// base addresses of flash and SRAM memories
#define FLASH_BASEADDR  0x08000000U  // U MEANS UNSIGNED INTEGER
#define SRAM1_BASEADDR  0x20000000U
// #define SRAM2_BASEADDR   // STM32F401RE DOES NOT HAVE SRAM2
#define ROM_BASEADDR 0x1FFF0000U
#define SRAM SRAM1_BASEADDR


// APBx AHBx Buses peripheral base address
#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

// base addresses  of peripherals which are hanging on AHB1 Bus
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR+0x1000)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR+0x1C00)
#define CRC_BASEADDR   (AHB1PERIPH_BASEADDR+0x3000)
#define RCC_BASEADDR   (AHB1PERIPH_BASEADDR+0x3800)
#define FIR_BASEADDR   (AHB1PERIPH_BASEADDR+0x3C00)
#define DMA1_BASEADDR  (AHB1PERIPH_BASEADDR+0x6000)
#define DMA2_BASEADDR  (AHB1PERIPH_BASEADDR+0x6400)

// base addresses  of peripherals which are hanging on APB1 Bus
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR+ 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR+ 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR+ 0x5C00)
#define USART2_BASEADDR (APB1PERIPH_BASEADDR+ 0x4400)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR+ 0x3C00)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR+ 0x3800)
#define TIM2_BASEADDR (APB1PERIPH_BASEADDR+ 0x0C00)
#define TIM3_BASEADDR (APB1PERIPH_BASEADDR+ 0x0800)
#define TIM4_BASEADDR (APB1PERIPH_BASEADDR+ 0x0400)
#define TIM5_BASEADDR (APB1PERIPH_BASEADDR+ 0x0000)

// base addresses  of peripherals which are hanging on APB2 Bus
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR+ 0x3C00)
#define SDIO_BASEADDR (APB2PERIPH_BASEADDR+ 0x2C00)
#define TIM1_BASEADDR (APB2PERIPH_BASEADDR+ 0x0000)
#define TIM9_BASEADDR (APB2PERIPH_BASEADDR+ 0x4000)
#define TIM10_BASEADDR (APB2PERIPH_BASEADDR+ 0x4400)
#define TIM11_BASEADDR (APB2PERIPH_BASEADDR+ 0x4800)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR+ 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR+ 0x1400)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR+ 0x3000)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR+ 0x3400)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR+ 0x3800)

/*
*    PERIPHERAL REGISTER DEFINITION STRUCTURES
*/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct{
	__vo uint32_t MODER;   //GPIO port mode register   Address offset: 0x00
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

/*
 * peripheral register definition structure for spi
 */
typedef struct{
	__vo uint32_t CR1;   //TODO   Address offset: 0x00
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;
/*
 * peripheral register definition structure for RCC
 */
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	     uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	     uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	     uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	     uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
         uint32_t RESERVED4[2];
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
         uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
         uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLL12SCFGR;
         uint32_t RESERVED7;
    __vo uint32_t DCKCFGR;

}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct{
	__vo uint32_t IMR;   //GIVE A SHORT DESCRIPTION   Address offset: 0x00
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct{
	__vo uint32_t MEMRMP;   //GIVE A SHORT DESCRIPTION   Address offset: 0x00
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 *  peripheral register definition structure for I2C
 */
typedef struct{
  __vo uint32_t CR1;        /*     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*     										Address offset: 0x0C */
  __vo uint32_t DR;         /*     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*     										Address offset: 0x24 */
}I2C_RegDef_t;

typedef struct{
	__vo uint32_t SR;         /*     										Address offset: 0x00 */
	__vo uint32_t DR;         /*     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*     										Address offset: 0x18 */
} USART_RegDef_t;


// peripheral definitions(peripheral base address typecasted to xxx_RegDef_t)
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define SPI1  ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  ((SPI_RegDef_t*)SPI4_BASEADDR)
#define RCC   ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define I2C1  ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6  ((USART_RegDef_t*)USART6_BASEADDR)


// CLOCK ENABLE MACROS FOR SYSCFG peripherals
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<14))

// CLOCK ENABLE mACROS FOR GPIOx peripherals
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))  // MEANING OF THIS IS I AM MAKING 7TH BIT AS 1

// CLOCK ENABLE MACROS FOR I2Cx peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1<<23))

// CLOCK ENABLE MACROS FOR SPIx peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<13))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))

// CLOCK ENABLE MACROS FOR USARTx peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB1ENR |= (1 << 5))


// CLOCK DISABLE MACROS FOR SYSCFG peripherals
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))


// CLOCK Disable MACROS FOR GPIOx peripherals
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1<<7))  // MEANING OF THIS IS I AM MAKING 7TH BIT AS 0

// CLOCK DISABLE MACROS FOR I2Cx peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1<<23))

// CLOCK DISABLE MACROS FOR SPIx peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1<<13))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1<<15))

// CLOCK DISABLE MACROS FOR USARTx peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI() (RCC->APB1ENR &= ~(1 << 5))



/*
 * Macros to reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<1));(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<2));(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<3));(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<4));(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<7));(RCC->AHB1RSTR &= ~(1<<7));}while(0)
// some generic macros
#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET
#define FLAG_RESET        RESET
#define FLAG_SET          SET



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
 * IRQ(Interrupt Request)Number of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: you may complete this list for other peripherals
 * these no is same for all mcu of  f4 family
 */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI5_9    23
#define IRQ_NO_EXTI10_15  40
#define IRQ_NO_SPI1		  35
#define IRQ_NO_SPI2       36
#define IRQ_NO_SPI3       51
#define IRQ_NO_SPI4       84
#define IRQ_NO_I2C1_EV    31
#define IRQ_NO_I2C1_ER    32
#define IRQ_NO_I2C2_EV    33
#define IRQ_NO_I2C2_ER    34
#define IRQ_NO_I2C3_EV    79
#define IRQ_NO_I2C3_ER    80
/*
 * Macros for all the possible priority levels
 * Macros for nvic irq priority no
 */
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15


#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_usart_driver.h"
#include "stm32f401xx_rcc_driver.h"

#endif /* INC_STM32F401XX_H_ */

