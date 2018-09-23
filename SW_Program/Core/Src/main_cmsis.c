


#include "main_cmsis.h"
#include "board_cmsis.h"


#define ERROR_CSS 0x01
#define ERROR_HSE_LOST 0x02
#define ERROR_UNEXPECTED_RCC_IRQ 0x04



volatile uint16_t error = 0xFF;
volatile uint32_t timestamp = 0, delay = 0;

uint8_t led = 0;


void StartHSE(void);
void  ConfigureGPIO(void);


int main(void) {

    SysTick_Config(8000);
    StartHSE();

    CM_ConfigureGPIO();

    CM_ADC_Init();

    while(1) {

        if(delay < timestamp) {

            delay = timestamp + 50;

            led = !led;

            if(led) {
                LL_GPIO_SetOutputPin(GPIOB, LED2_PIN);
                LL_GPIO_ResetOutputPin(GPIOB, LED6_PIN);
            } else {
                LL_GPIO_SetOutputPin(GPIOB, LED6_PIN);
                LL_GPIO_ResetOutputPin(GPIOB, LED2_PIN);
            }


        }
    }
}




/*  */
__INLINE void StartHSE(void) {

    NVIC_EnableIRQ(RCC_CRS_IRQn);
    NVIC_SetPriority(RCC_CRS_IRQn,0);

    RCC->CIR |= RCC_CIR_HSERDYIE;
    RCC->CR |= RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON;
}



/*  */
void NMI_Handler(void) {
    if ((RCC->CIR & RCC_CIR_CSSF) != 0) {
        error = ERROR_CSS; /* Report the error */
        RCC->CIR |= RCC_CIR_CSSC; /* Clear the flag */
    }
}

/*  */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/*  */
void SVC_Handler(void) {
}

/*  */
void PendSV_Handler(void) {
}


/*  */
void SysTick_Handler(void) {
    timestamp++;
}


/*  */
void RCC_CRS_IRQHandler(void) {

    if ((RCC->CIR & RCC_CIR_HSERDYF) != 0) {                          /* Check the flag HSE ready */
        RCC->CIR |= RCC_CIR_HSERDYC;                                    /* Clear the flag HSE ready */
        RCC->CFGR = ((RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_0);     /* Switch the system clock to HSE */
    } else {
        error = ERROR_UNEXPECTED_RCC_IRQ; /* Report an error */
    }
}

