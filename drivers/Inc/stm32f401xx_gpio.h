/*
 * stm32f401xx_gpio.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Ch.vivek
 */

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_

#include "stm32f401xx.h"


/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOType;
	uint8_t GPIO_PUPDCtrl;
	uint8_t GPIO_ALTFunMode;
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t  *pGPIOx;                   /*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;          /*!< This holds GPIO pin configuration settings >*/
} GPIO_Handle_t;


#endif /* INC_STM32F401XX_GPIO_H_ */
