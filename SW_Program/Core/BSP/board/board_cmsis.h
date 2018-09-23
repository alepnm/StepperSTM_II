#ifndef BOARD_CMSIS_H_INCLUDED
#define BOARD_CMSIS_H_INCLUDED


#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"

#define LED2_PIN    LL_GPIO_PIN_1
#define LED5_PIN    LL_GPIO_PIN_0
#define LED6_PIN    LL_GPIO_PIN_10
#define LED7_PIN    LL_GPIO_PIN_2



void        CM_ConfigureGPIO(void);


void        CM_ADC_Init(void);

void        ADC1_COMP_IRQHandler(void);


#endif /* BOARD_CMSIS_H_INCLUDED */
