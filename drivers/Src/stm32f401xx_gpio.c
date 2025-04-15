/*
 * stm32f401xx_gpio.c
 *
 *  Created on: Apr 11, 2025
 *      Author: Admin
 */
#include "stm32f401xx_gpio.h"

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

}


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

}

void GPIO_IRQHandling(uint8_t PinNumber){

}

