
#include "board_cmsis.h"







__INLINE void  CM_ConfigureGPIO(void) {

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    LL_GPIO_SetPinMode(GPIOB, LED2_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LED5_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LED6_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LED7_PIN, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetPinOutputType(GPIOB, LED2_PIN|LED5_PIN|LED6_PIN|LED7_PIN, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinSpeed(GPIOB, LED2_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LED5_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LED6_PIN, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LED7_PIN, LL_GPIO_SPEED_FREQ_HIGH);

    LL_GPIO_SetPinPull(GPIOB, LED2_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOB, LED5_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOB, LED6_PIN, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinPull(GPIOB, LED7_PIN, LL_GPIO_PULL_NO);

}






/*  */
__INLINE void CM_ADC_Init(void) {

//    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                             /* Enable the peripheral clock of the ADC */
//    RCC->CR2 |= RCC_CR2_HSI14ON;                                    /* Start HSI14 RC oscillator */
//    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);                     /* Wait HSI14 is ready */
//
//    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                              /* Enable the peripheral clock of GPIOA */
//    GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1;          /* Select analog mode for PA0, PA1 */
//
//    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;                               /* Select HSI14 by writing 00 in CKMODE (reset value) */
//    ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;                                /* Select the auto off mode */
//    //ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1;           /* Select CHSEL0, CHSEL1 for VRefInt */
//    ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
//    ADC->CCR |= ADC_CCR_VREFEN;                                     /* Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) */




    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_RCC_HSI14_Enable();
    while(LL_RCC_HSI14_IsReady() == 0);

    LL_RCC_HSI14_EnableADCControl();


    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);

    LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(), LL_ADC_PATH_INTERNAL_VREFINT);

    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV2);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_10B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_55CYCLES_5);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);




    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1) != 0);

    LL_ADC_Disable(ADC1);

    NVIC_EnableIRQ(ADC1_COMP_IRQn);                                 /* Enable Interrupt on ADC */
    NVIC_SetPriority(ADC1_COMP_IRQn,0);                             /* Set priority for ADC */
}


/*  */
void ADC1_COMP_IRQHandler(void) {

}



