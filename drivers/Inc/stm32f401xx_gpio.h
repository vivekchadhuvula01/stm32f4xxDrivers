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



/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_0    0
#define GPIO_PIN_1    1
#define GPIO_PIN_2    2
#define GPIO_PIN_3    3
#define GPIO_PIN_4    4
#define GPIO_PIN_5    5
#define GPIO_PIN_6    6
#define GPIO_PIN_7    7
#define GPIO_PIN_8    8
#define GPIO_PIN_9    9
#define GPIO_PIN_10    10
#define GPIO_PIN_11    11
#define GPIO_PIN_12    12
#define GPIO_PIN_13    13
#define GPIO_PIN_14    14
#define GPIO_PIN_15    15



/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN          0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG  3
#define GPIO_MODE_IT_FT      4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEE_LOW  0
#define GPIO_SPEE_MEDIUM  1
#define GPIO_SPEE_FAST  2
#define GPIO_SPEE_HIGH   3


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   0
#define GPIO_PIN_PU      1
#define GPIO_PIN_PD      2


// write for interrupts
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClkCntrl(void);


/*
 * Init and De-init
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data read and write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_TOggleOutputPin(void);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(void);
void GPIO_IRQPriorityCOnfig(void);
void GPIO_IRQHandling(void);


#endif /* INC_STM32F401XX_GPIO_H_ */
