
#include "board.h"
#include "user_mb_app.h"


UART_HandleTypeDef* ports[] = {&huart1, NULL, NULL};
static const uint32_t baudrates[6u] = {2400u, 4800u, 9600u, 19200u, 38400u, 57600u};

/* Functions prototypes */



/*  */
void BSP_HW_Init(void) {



    BSP_PwmTimerInit();

}

/*  */
void BSP_Delay(uint32_t delay) {
    HAL_Delay(delay);
}


/*****************************************************************************************
* Function Name: UartConfig
******************************************************************************************
* Summary:
*  Si funkcija konfiguruoja UART nustatymus. Patikrina parametra, ar jie yra leistini
*  ar ne. Jai yra klaidingu parametru, jie uzkeiciami defaultiniais
*
* Parametrai:
*   uint32_t    baudrate :  (uint32_t) baudraitas is standartiniu (4800 - 115200)
*   uint8_t     parity  :   (uint32_t) UART_PARITY_NONE/UART_PARITY_EVEN/UART_PARITY_ODD
*   uint8_t     stopbits :  (uint32_t) UART_STOPBITS_1/UART_STOPBITS_1_5/UART_STOPBITS_2
*               databits :  (uint32_t) 8/9
*
* Grazina: Statusa HAL_StatusTypeDef
*****************************************************************************************/
HAL_StatusTypeDef BSP_UartConfig( uint8_t ucPORT, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity ) {

    if( CheckBaudrateValue( ulBaudRate ) == HAL_ERROR ) {
        ulBaudRate = GetBaudrateByIndex( MBBAURATE_DEF );
        usRegHoldingBuf[HR_MBBAUDRATE] = MBBAURATE_DEF;
    }

    if(ports[ucPORT] == NULL) return HAL_ERROR;

    ports[ucPORT]->Init.BaudRate = ulBaudRate;

    switch (ucDataBits) {
    case 9:
        //MbPortUART->Init.WordLength = UART_WORDLENGTH_9B;
        ports[ucPORT]->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        ports[ucPORT]->Init.WordLength = UART_WORDLENGTH_8B;
    }

    switch(eParity) {
    case MB_PAR_ODD:
        ports[ucPORT]->Init.Parity = UART_PARITY_ODD;
        ports[ucPORT]->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    case MB_PAR_EVEN:
        ports[ucPORT]->Init.Parity = UART_PARITY_EVEN;
        ports[ucPORT]->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        ports[ucPORT]->Init.Parity = UART_PARITY_NONE;
        ports[ucPORT]->Init.WordLength = UART_WORDLENGTH_8B;
    }

    return HAL_OK;
}


/*******************************************************************************
* Function Name: UartStart
********************************************************************************
* Summary: Funkcija startuoja UART irengini.
*
* Parametrai: pointeris i UART objekta UART_HandleTypeDef*
*
* Grazina: Statusa HAL_StatusTypeDef
*
*******************************************************************************/
HAL_StatusTypeDef BSP_UartStart( uint8_t ucPORT ) {

    if(ports[ucPORT] == NULL) return HAL_ERROR;

    if( HAL_UART_DeInit(ports[ucPORT]) != HAL_OK ) Error_Handler();

    if( ports[ucPORT] == ports[MbPortParams.Uart] ) {
        if( HAL_RS485Ex_Init(ports[ucPORT], UART_DE_POLARITY_HIGH, 0, 0) !=  HAL_OK ) Error_Handler();
    }

    if( HAL_UART_Init(ports[ucPORT]) != HAL_OK ) Error_Handler();

    vMBPortSerialEnable( TRUE, FALSE );

    return HAL_OK;
}


/*****************************************************************************************
* Function Name: UartStop
******************************************************************************************
* Summary: Si funkcija stabdo UART moduli.
*
* Parametrai: pointeris i UART objekta UART_HandleTypeDef*.
*
* Grazina: Statusa HAL_StatusTypeDef
*****************************************************************************************/
HAL_StatusTypeDef BSP_UartStop( uint8_t ucPORT ) {

    if(ports[ucPORT] == NULL) return HAL_ERROR;

    if( ports[ucPORT] == ports[MbPortParams.Uart] ) {
        if( HAL_UART_AbortReceive_IT(ports[ucPORT]) != HAL_OK ) Error_Handler();
    }

    return HAL_OK;
}


/* chekinam bodreito reiksme - ar standartine? */
HAL_StatusTypeDef CheckBaudrateValue(uint32_t baudrate) {

    if( GetIndexByBaudrate( baudrate ) == 0xFF ) return HAL_ERROR;

    return HAL_OK;
}


HAL_StatusTypeDef CheckBaudrateIndex( uint8_t idx ) {

    if( GetBaudrateByIndex( idx ) == 0xFFFFFFFF ) return HAL_ERROR;

    return HAL_OK;
}


/* grazinam bodreito indeksa lenteleje. Jai bodreito reiksme nestandartine grazinam 0xFF */
uint8_t GetIndexByBaudrate( uint32_t baudrate ) {

    uint8_t i = 0;

    while(baudrate != baudrates[i]) {
        if( i >= ( sizeof(baudrates)/sizeof(baudrate) ) ) {
            i = 0xFF;
            break;
        }

        i++;
    }

    return i;
}


/* grazinam bodreita pagal jo indeksa lenteleje. Jai indeksas didesnis uz standartiniu bodreitu skaiciu,
grazinam 0xFFFFFFFF */
uint32_t GetBaudrateByIndex( uint8_t idx ) {

    if( idx > sizeof(baudrates)/sizeof(uint32_t) ) return 0xFFFFFFFF;

    return baudrates[idx];
}


uint8_t GetCurrentBaudrateIndex( void ) {
    return GetIndexByBaudrate( ports[MbPortParams.Uart]->Init.BaudRate );
}

uint8_t GetCurrentParity( void ) {
    if(ports[MbPortParams.Uart]->Init.Parity == UART_PARITY_ODD) return MB_PAR_ODD;
    if(ports[MbPortParams.Uart]->Init.Parity == UART_PARITY_EVEN) return MB_PAR_EVEN;
    return MB_PAR_NONE;
}

uint8_t GetCurrentStopBits( void ) {
    if(ports[MbPortParams.Uart]->Init.StopBits == UART_STOPBITS_2) return 2U;
    return 1U;
}

uint8_t GetCurrentDataBits( void ) {
    if(ports[MbPortParams.Uart]->Init.WordLength == UART_WORDLENGTH_9B) return 9U;
    return 8U;
}





///* UART ISR Handler */
void BSP_UART_IRQ_Handler(USART_TypeDef * usart) {

    if (usart == ports[MbPortParams.Uart]->Instance) {

        if( (__HAL_UART_GET_IT(ports[MbPortParams.Uart], UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ports[MbPortParams.Uart], UART_IT_RXNE) != RESET) ) {

            //prvvUARTRxISR( );
            pxMBFrameCBByteReceived();
        }

        if( (__HAL_UART_GET_IT(ports[MbPortParams.Uart], UART_IT_TXE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ports[MbPortParams.Uart], UART_IT_TXE) != RESET) ) {

            //prvvUARTTxReadyISR( );
            pxMBFrameCBTransmitterEmpty();
        }
    }
}


/*  */
void BSP_MbPortTimerInit(uint16_t period) {

    htim6.Init.Prescaler = (uint32_t)(SystemCoreClock / 48000) - 1;
    htim6.Init.Period = (uint32_t)( period - 1 );
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

    if ( HAL_TIM_Base_Init(&htim6) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if ( HAL_TIM_OnePulse_Start_IT(&htim6, TIM_CHANNEL_ALL) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }

    //__HAL_TIM_ENABLE_IT(pModbusTimer, TIM_IT_UPDATE);
}


/*  */
void TIM6_DAC1_IRQHandler(void) {

    if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {

        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

        ( void )pxMBPortCBTimerExpired( );

        //HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
    }
}



/* PWM formavimo taimerio inicializavimas */
void BSP_PwmTimerInit(void) {

    TIM_OC_InitTypeDef sConfigOC = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 500,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };


    htim17.Instance = TIM17;
    htim17.Init.Prescaler = ( HAL_RCC_GetPCLK1Freq() / 1000000 ) - 1;;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 1000;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Init(&htim17) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}


/* Formuoja 0-10V isejime itampa, atitinkamos parametrui reiksmes
Parametrai: volts -> 0-10 (Voltai)
*/
void BSP_SetAnalogOut(float volts) {

    volts = (volts < 0) ? 0 : volts;
    volts = (volts > 10.00) ? 10.00 : volts;

    __HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, volts * 75.5);
}


/* Formuoja DAC isejime itampa
Parametrai: val -> 0-100 (procentai)
*/
void BSP_SetDAC_Value(float dac) {

    dac = (dac < 0) ? 0 : dac;
    dac = (dac > 100) ? 100 : dac;

    if( HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, (uint32_t)(dac * 2.55) ) != HAL_OK) Error_Handler();
}


/*  */
void BSP_ReadDipSwitch(void) {

    uint8_t delay = 10;

    HAL_GPIO_WritePin(HC165_LATCH_GPIO_Port, HC165_LATCH_Pin, GPIO_PIN_RESET);
    while(delay--);
    delay = 10;
    HAL_GPIO_WritePin(HC165_LATCH_GPIO_Port, HC165_LATCH_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(HCCTRL_GPIO_Port, HCCTRL_Pin, GPIO_PIN_RESET);
    while(delay--);
    delay = 10;
    HAL_GPIO_WritePin(HCCTRL_GPIO_Port, HCCTRL_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(HC165_SS_GPIO_Port, HC165_SS_Pin, GPIO_PIN_RESET);
    //(void)HAL_SPI_Receive(&hspi1, &(SMC_Control.DipSwitch.Data), 1, 10);
    BSP_SpiRx(&(SMC_Control.DipSwitch.Data), 1);
    HAL_GPIO_WritePin(HC165_SS_GPIO_Port, HC165_SS_Pin, GPIO_PIN_SET);
}



/*  */
HAL_StatusTypeDef BSP_SpiTx(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Transmit(&hspi1, pData, len, 10) )!= HAL_OK ) return result;
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return HAL_OK;
}

/*  */
HAL_StatusTypeDef BSP_SpiRx(uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Receive(&hspi1, pData, len, 10) ) != HAL_OK ) return result;
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return HAL_OK;
}

/*  */
HAL_StatusTypeDef BSP_SpiRxTx(uint8_t* pData, uint8_t len) {

    return HAL_OK;
}



/* [] END OF FILE */
