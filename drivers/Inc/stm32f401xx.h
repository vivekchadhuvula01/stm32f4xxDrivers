/*
 * stm32f407xx.h
 *
 *  Created on: Apr 8, 2025
 *      Author: Ch.Vivek
 */



/*
 * Register Addr = Base Addr + Offset
 */
#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__ ((weak))


//DEFINE VARIOUS ADDRESSES OF MEMORY LOCATIONS
#define FLASH_BASE_ADDR     0x08000000U
#define SRAM1_BASE_ADDR    0x20000000U
#define ROM_BASE_ADDR        0x1FFF0000U
#define SRAM 	                        SRAM1_BASE_ADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR                   0x40000000U
#define APB1PERIPH_BASEADDR           PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR           0x40010000U
#define AHB1PERIPH_BASEADDR          0x40020000U
#define AHB2PERIPH_BASEADDR          0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus

 */
#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
//#define GPIOF_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
//#define GPIOG_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR              (AHB1PERIPH_BASEADDR + 0x3800)


/*
 * Base addresses of peripherals which are hanging on APB1 bus

 */

#define I2C1_BASEADDR	          (APB1PERIPH_BASEADDR +  0x5400)
#define I2C2_BASEADDR	          (APB1PERIPH_BASEADDR +  0x5800)
#define I2C3_BASEADDR	          (APB1PERIPH_BASEADDR +  0x5C00)

#define SPI2_BASEADDR	          (APB1PERIPH_BASEADDR +  0x3800)
#define SPI3_BASEADDR	          (APB1PERIPH_BASEADDR +  0x3C00)

#define USART2_BASEADDR      (APB1PERIPH_BASEADDR +  0x4400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus

 */

#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR        (APB2PERIPH_BASEADDR + 0x3800)
#define USART6_BASEADDR           (APB2PERIPH_BASEADDR + 0x1400)
#define USART1_BASEADDR           (APB2PERIPH_BASEADDR + 0x1000)


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU

 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

//gpio registers
typedef struct {
	__vo uint32_t MODER;     //Address offset: 0x00
	__vo uint32_t OTYPER;    // Address offset: 0x04
	__vo uint32_t OSPEEDR; //Address offset: 0x08
	__vo uint32_t PUPDR;    //Address offset: 0x0C
	__vo uint32_t IDR;        //Address offset: 0x10
	__vo uint32_t ODR;        //Address offset: 0x14
	__vo uint32_t BSRR;        //Address offset: 0x18
	__vo uint32_t LCKR;        //Address offset: 0x1C
	__vo uint32_t AFR[2];	/*!< AFR[0] : GPIO alternate function low register,
	                                       AF[1] : GPIO alternate function high register
	                                       Address offset: 0x20-0x24 */
}  GPIO_RegDef_t;

//rcc registers
typedef struct {
	__vo uint32_t RC;    // Address offset: 0x00
	__vo uint32_t PLLCFGR;   // Address offset: 0x04
	__vo uint32_t CFGR;  // Address offset: 0x08
	__vo uint32_t CIR;  // Address offset: 0x0C
	__vo uint32_t AHB1RSTR;  // Address offset: 0x10
	__vo uint32_t AHB2RSTR;  // Address offset: 0x14
	__vo uint32_t APB1RSTR;  // Address offset: 0x20
	__vo uint32_t APB2RSTR;  // Address offset: 0x24
	__vo uint32_t AHB1ENR;  // Address offset: 0x30
	__vo uint32_t AHB2ENR; // Address offset: 0x34
	__vo uint32_t APB1ENR;  // Address offset: 0x40
	__vo uint32_t APB2ENR;  // Address offset: 0x44
	__vo uint32_t AHB1LPENR; // Address offset: 0x50
	__vo uint32_t AHB2LPENR; // Address offset: 0x54
	__vo uint32_t APB1LPENR; // Address offset: 0x60
	__vo uint32_t APB2LPENR; //Address offset: 0x64
	__vo uint32_t BDCR;  // Address offset: 0x70
	__vo uint32_t CSR;  // Address offset: 0x74
	__vo uint32_t SSCGR;  // Address offset: 0x80
	__vo uint32_t PLLI2SCFGR; //Address offset: 0x84
	__vo uint32_t DCKCFGR; //Address offset: 0x8C



} RCC_RegDef_t;


// extI registers
typedef struct{
	__vo uint32_t  IMR; //0x00
	__vo uint32_t  EMR;//0x04
	__vo uint32_t  RTSR;//0x08
	__vo uint32_t  FTSR;//0x0C
	__vo uint32_t  SWIER;//0x10
	__vo uint32_t  PR;//0x14

} EXTI_RegDef_t;


//spi registers
typedef struct
{
	__vo uint32_t CR1; //0x00
	__vo uint32_t CR2; //0x04
	__vo uint32_t DR; //0x0x0C
	__vo uint32_t CRCPR; // Address offset: 0x10     not used in I2S (mode)
	__vo uint32_t RXCRCR; //0x14
	__vo uint32_t TXCRCR; //0x18
	__vo uint32_t I2SCFGR; //0x1C
	__vo uint32_t I2SPR; // 0x20

} SPI_RegDef_t;

//syscfg registers
typedef struct {
	__vo uint32_t MEMRMP; //Address offset: 0x00
	__vo uint32_t PMC;  // Address offset: 0x04
	__vo uint32_t EXTICR1; //Address offset: 0x08
	__vo uint32_t EXTICR2; // Address offset: 0x0C
	__vo uint32_t EXTICR3; //Address offset: 0x10
	__vo uint32_t EXTICR4; //Address offset: 0x14
	__vo uint32_t CMPCR; //Address offset: 0x20


}SYSCFG_RegDef_t;

// i2c registers
typedef struct {
	__vo uint32_t CR1; //Address offset: 0x00
	__vo uint32_t CR2; //Address offset: 0x04
	__vo uint32_t OAR1; // Address offset: 0x08
	__vo uint32_t OAR2; //0x0C
	__vo uint32_t DR; // 0x10
	__vo uint32_t SR1; //0x14
	__vo uint32_t SR2; //0x18
	__vo uint32_t CCR; //0x1C
	__vo uint32_t TRISE;//0x20
	__vo uint32_t FLTR;//0x24

}I2C_RegDef_t;


// usart registers
typedef struct {
	__vo uint32_t SR; //0x00
	__vo uint32_t DR; //0x04
	__vo uint32_t BRR;//0x08
	__vo uint32_t CR1;//0x0C
	__vo uint32_t CR2;//0x10
	__vo uint32_t CR3;//0x14
	__vo uint32_t GTPR;//0x18

}USART_RegDef_t;


/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 * Treat the address GPIOD_BASEADDR as a pointer to a GPIO_RegDef_t structure
 * So now, you can access registers like this:

			GPIOD->MODER = 0xA8000000;
			GPIOD->ODR |= (1 << 12);  // Set PD12 high
 */
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC       ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI      ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1       ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2       ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3       ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4       ((SPI_RegDef_t*)SPI4_BASEADDR)

#define USART1   ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2   ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6   ((USART_RegDef_t*)USART6_BASEADDR)

#define I2C1        ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2        ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3        ((I2C_RegDef_t*)I2C3_BASEADDR)


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART2_CLK_EN() (RCC->AHB1ENR |= (1 << 17))
#define USART1_CLK_EN() (RCC->AHB2ENR |= (1 << 4))
#define USART6_CLK_EN() (RCC->AHB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for I2Cx peripherals
 * TODO
 */
#define I2C1_CLK_EN() (RCC->APB1ENR |= (1<<21))
#define I2C2_CLK_EN() (RCC->APB1ENR |= (1<<22))
#define I2C3_CLK_EN() (RCC->APB1ENR |= (1<<23))



/*
 * Clock Enable Macros for SPIx peripherals
 * TODO
 */
#define SPI1_CLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN() (RCC->APB1ENR |= (1<<15))
#define SPI4_CLK_EN() (RCC->APB2ENR |= (1<<13))



/*
 * Clock Enable Macros for SYSCFG peripheral
 * TODO
 */
#define SYSCFG_CLK_EN() (RCC->APB2ENR |= (1<<14))

//ADD NECESSARY GPIO PINS



#endif /* INC_STM32F401XX_H_ */
