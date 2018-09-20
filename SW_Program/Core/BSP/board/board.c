
#include "board.h"
#include "user_mb_app.h"
#include "sound.h"


#define TEMP110_CAL_ADDR                ( (uint16_t*) ((uint32_t) 0x1FFFF7C2) )
#define TEMP30_CAL_ADDR                 ( (uint16_t*) ((uint32_t) 0x1FFFF7B8) )
#define VDD_CALIB                       ( (uint16_t) (330) )
#define VDD_APPLI                       ( (uint16_t) (300) )


UART_HandleTypeDef* ports[] = {&huart1, NULL, NULL};


/* Functions prototypes */
extern HAL_StatusTypeDef    CheckBaudrateValue(uint32_t baudrate);
extern HAL_StatusTypeDef    CheckBaudrateIndex( uint8_t idx );
extern uint32_t             GetBaudrateByIndex( uint8_t idx );


static HAL_StatusTypeDef    BSP_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint8_t len);
static HAL_StatusTypeDef    BSP_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint8_t len);
static HAL_StatusTypeDef    BSP_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pDataTx, uint8_t* pDataRx, uint8_t len);

static HAL_StatusTypeDef    ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
static uint16_t             GetAdcValue(ADC_ChannelConfTypeDef* sConfig);

/*  */
void BSP_HW_Init(void) {

    M25AA_CS_HIGH();

    HC598_LAT_HIGH();
    HC598_CTRL_HIGH();
    HC598_CS_HIGH();

    L6470_CS_HIGH();
    L6470_RST_HIGH();

    STATUS_LED_OFF();
    FAULT_LED_OFF();
    COOLER_OFF();
    RELAY_OFF();

        /* MCU periferijos inicializavimas */
    while(HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK);

    BSP_PwmTimerInit();

    (void)HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}


/*  */
HAL_StatusTypeDef BSP_UartConfig( uint8_t ucPORT, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity ) {

    if( CheckBaudrateValue( ulBaudRate ) == HAL_ERROR ) {
        ulBaudRate = GetBaudrateByIndex( MBBAURATE_DEF );
        usRegHoldingBuf[HR_MBBAUDRATE] = MBBAURATE_DEF;
    }

    if(ports[ucPORT] == NULL) return HAL_ERROR;

    ports[ucPORT]->Init.BaudRate = ulBaudRate;

    switch (ucDataBits) {
    case 9:
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


/*  */
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


/*  */
HAL_StatusTypeDef BSP_UartStop( uint8_t ucPORT ) {

    if(ports[ucPORT] == NULL) return HAL_ERROR;

    if( ports[ucPORT] == ports[MbPortParams.Uart] ) {
        if( HAL_UART_AbortReceive_IT(ports[ucPORT]) != HAL_OK ) Error_Handler();
    }

    return HAL_OK;
}


///* UART ISR Handler */
void BSP_UART_IRQ_Handler(USART_TypeDef * usart) {

    if (usart == ports[MbPortParams.Uart]->Instance) {

        if( (__HAL_UART_GET_IT(ports[MbPortParams.Uart], UART_IT_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ports[MbPortParams.Uart], UART_IT_RXNE) != RESET) ) {

            //prvvUARTRxISR( );
            (void)pxMBFrameCBByteReceived();
        }

        if( (__HAL_UART_GET_IT(ports[MbPortParams.Uart], UART_IT_TXE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ports[MbPortParams.Uart], UART_IT_TXE) != RESET) ) {

            //prvvUARTTxReadyISR( );
            (void)pxMBFrameCBTransmitterEmpty();
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

    //__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}


/*  */
void BSP_MbPortTimerEnable(void) {

    __HAL_TIM_SET_COUNTER(&htim6, 0);

    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

    if ( HAL_TIM_Base_Start(&htim6) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/*  */
void BSP_MbPortTimerDisable(void) {

    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);

    if ( HAL_TIM_Base_Stop(&htim6) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/*  */
void TIM6_DAC1_IRQHandler(void) {

    if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {

        __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

        (void)pxMBPortCBTimerExpired( );
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



/*  */
void BSP_SoundTimerInit(void) {

    TIM_OC_InitTypeDef sConfigOC = {
        .OCMode = TIM_OCMODE_PWM1,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET
    };

    htim16.Init.Prescaler = (SystemCoreClock / 1000000) - 1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = (1000000 / Sounder.freq) - 1;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim16) != HAL_OK){
            Error_Handler();
    }

    sConfigOC.Pulse = (htim16.Init.Period * Sounder.volume) / 100;

    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){
            Error_Handler();
    }
}


/* */
void BSP_SoundTimerStart(void) {
    if (HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

/*  */
void BSP_SoundTimerStop(void) {
    if (HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1) != HAL_OK) {
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

    HC598_LAT_LOW();
    while(delay--);
    delay = 10;
    HC598_LAT_HIGH();

    HC598_CTRL_LOW();
    while(delay--);
    delay = 10;
    HC598_CTRL_HIGH();

    HC598_CS_LOW();
    BSP_SPI_Receive(&hspi1, &(SMC_Control.DipSwitch.Data), 1);
    HC598_CS_HIGH();
}



/* Suvidurkintus rezultatus sudedam i tam skirtus registrus */
void BSP_ReadAnalogInputs(void) {

    ADC_ChannelConfTypeDef sConfig = {
        .Channel = ADC_CHANNEL_0,
        .Rank = ADC_RANK_CHANNEL_NUMBER,
        .SamplingTime = ADC_SAMPLETIME_55CYCLES_5
    };

    static uint8_t stage = 0, n_spreq = 0, n_vbus = 0, n_itemp = 0;
    static uint32_t sum_spreq = 0, sum_vbus = 0, sum_itemp = 0;

    __enter_critical();

    switch(stage) {
    case 0:

        if(n_vbus++ < 64) sum_vbus += GetAdcValue(&sConfig);
        else {

            SMC_Control.ADC_Vals.Vbus = (uint16_t)(sum_vbus>>6);
            usRegInputBuf[IR_VBUS_VALUE] = SMC_Control.ADC_Vals.Vbus * 0.822;   // verciam voltais  ( formatas V*100 )
            n_vbus = sum_vbus = 0;
        }

        stage = 1;
        break;
    case 1:

        sConfig.Channel = ADC_CHANNEL_1;

        uint16_t adc = GetAdcValue(&sConfig);

        /* filtruojam triuksma ir vidurkinam ADC reiksme */
        if( SMC_Control.ADC_Vals.SpReq < adc - 20 || SMC_Control.ADC_Vals.SpReq > adc + 20 ) {

            if(n_spreq++ < 8) sum_spreq += adc;
            else {

                SMC_Control.ADC_Vals.SpReq = (uint16_t)(sum_spreq>>3);
                usRegInputBuf[IR_SPREQ_VALUE] = SMC_Control.ADC_Vals.SpReq * 0.235;   // verciam voltais  ( formatas V*100 )
                n_spreq = sum_spreq = 0;
            }
        }

        stage = 2;
        break;
    case 2:

        sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
        sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

        if(n_itemp++ < 8) sum_itemp += GetAdcValue(&sConfig);
        else {

            SMC_Control.ADC_Vals.McuTemp  = (int32_t) (sum_itemp>>3);

            int32_t temperature = ((SMC_Control.ADC_Vals.McuTemp * VDD_APPLI / VDD_CALIB) - (int32_t) *TEMP30_CAL_ADDR );
            temperature = temperature * (int32_t)(110 - 30);
            usRegInputBuf[IR_MCUTEMP] = (uint16_t)(temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR) + 30);

            n_itemp = sum_itemp = 0;
        }

        stage = 0;
        break;
    }

    __exit_critical();
}


/*  */
HAL_StatusTypeDef    BSP_L6470_SPI_TransmitReceive(uint8_t* pDataTx, uint8_t* pDataRx, uint8_t len){
    return BSP_SPI_TransmitReceive(&hspi1, pDataTx, pDataRx, len);
}

/*  */
HAL_StatusTypeDef    BSP_M25AAxx_SPI_Transmit(uint8_t* pData, uint8_t len){
    return BSP_SPI_Transmit(&hspi1, pData, len);
}

/*  */
HAL_StatusTypeDef    BSP_M25AAxx_SPI_Receive(uint8_t* pData, uint8_t len){
    return BSP_SPI_Receive(&hspi1, pData, len);
}



/* Vieno pasirinkto ADC kanalo nuskaitymas */
static uint16_t GetAdcValue(ADC_ChannelConfTypeDef* sConfig) {

    uint16_t val = 0;
    uint8_t i = 0;

    (void)ADC_ConfigChannel(&hadc, sConfig);

    for(i = 0; i < 2; i++) {

        (void)HAL_ADC_Start(&hadc);
        while(HAL_ADC_PollForConversion(&hadc,0) != HAL_OK);

        val += (uint16_t)HAL_ADC_GetValue(&hadc);
    }

    (void)HAL_ADC_Stop(&hadc);

    return (uint16_t)(val>>1);
}


/* Pataisyta HAL_ADC_ConfigChannel funkcija is HAL bibliotekos */
static HAL_StatusTypeDef ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig) {

    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    volatile uint32_t wait_loop_index = 0U;

    /* Check the parameters */
    assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));
    assert_param(IS_ADC_CHANNEL(sConfig->Channel));
    assert_param(IS_ADC_RANK(sConfig->Rank));

    if (! IS_ADC_SAMPLE_TIME(hadc->Init.SamplingTimeCommon)) {
        assert_param(IS_ADC_SAMPLE_TIME(sConfig->SamplingTime));
    }

    __HAL_LOCK(hadc);

    if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET) {
        hadc->Instance->CHSELR = ADC_CHSELR_CHANNEL(sConfig->Channel);

        if (! IS_ADC_SAMPLE_TIME(hadc->Init.SamplingTimeCommon)) {

            if (sConfig->SamplingTime != ADC_GET_SAMPLINGTIME(hadc)) {
                hadc->Instance->SMPR &= ~(ADC_SMPR_SMP);
                hadc->Instance->SMPR |= ADC_SMPR_SET(sConfig->SamplingTime);
            }
        }

        if(ADC_IS_CHANNEL_INTERNAL(sConfig->Channel)) {
            ADC->CCR |= ADC_CHANNEL_INTERNAL_PATH(sConfig->Channel);
            if (sConfig->Channel == ADC_CHANNEL_TEMPSENSOR) {
                wait_loop_index = ( 10 * (SystemCoreClock / 1000000U));
                while(wait_loop_index != 0U) {
                    wait_loop_index--;
                }
            }
        }
    } else {

        SET_BIT(hadc->State, HAL_ADC_STATE_ERROR_CONFIG);
        tmp_hal_status = HAL_ERROR;
    }

    __HAL_UNLOCK(hadc);

    return tmp_hal_status;
}


/*  */
void BSP_SystemReset(void){
    NVIC_SystemReset();
}



/*  */
static HAL_StatusTypeDef BSP_SPI_Transmit(SPI_HandleTypeDef* hspi, uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Transmit(hspi, pData, len, 10) )!= HAL_OK ) return result;
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return HAL_OK;
}

/*  */
static HAL_StatusTypeDef BSP_SPI_Receive(SPI_HandleTypeDef* hspi, uint8_t* pData, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( ( result = HAL_SPI_Receive(hspi, pData, len, 10) ) != HAL_OK ) return result;
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return HAL_OK;
}

/*  */
static HAL_StatusTypeDef BSP_SPI_TransmitReceive(SPI_HandleTypeDef* hspi, uint8_t* pDataTx, uint8_t* pDataRx, uint8_t len) {

    HAL_StatusTypeDef result = HAL_OK;

    if( (result = HAL_SPI_TransmitReceive(hspi, pDataTx, pDataRx, len, 10) ) != HAL_OK ) return result;
    while( hspi1.State == HAL_SPI_STATE_BUSY );

    return HAL_OK;
}



/*  */
HAL_StatusTypeDef BSP_IIC_IsReady(uint8_t base_addr){

    uint32_t timeout = HAL_GetTick() + 100;

    while ( HAL_I2C_IsDeviceReady(&hi2c1, (base_addr<<1), 3, 10) != HAL_OK );
    while ( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){
        if( timeout < HAL_GetTick() ) return HAL_TIMEOUT;    //eeprom fault!!!
    }

    return HAL_OK;
}

/*  */
HAL_StatusTypeDef BSP_IIC_Read(uint16_t eeaddr, uint16_t mem_addr, uint8_t* pData, uint16_t len){
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(&hi2c1, eeaddr, mem_addr, (uint16_t)I2C_MEMADD_SIZE_8BIT, pData, len, 10);
    while ( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY );
    return result;
}

/*  */
HAL_StatusTypeDef BSP_IIC_Write(uint16_t eeaddr, uint16_t mem_addr, uint8_t* pData, uint16_t len){
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(&hi2c1, eeaddr, mem_addr, (uint16_t)I2C_MEMADD_SIZE_8BIT, pData, len, 10);
    while ( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY );
    return result;
}

/* [] END OF FILE */
