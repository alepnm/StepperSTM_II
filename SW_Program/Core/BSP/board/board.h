#ifndef BOARD_H_INCLUDED
#define BOARD_H_INCLUDED

#include "stm32f0xx_hal.h"
#include "common.h"
#include "hw_typedefs.h"
#include "systick.h"


#define __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define __exit_critical() __set_PRIMASK(irq);}
#define ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

#define ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
#define EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}

#define LED_ON                      GPIO_PIN_RESET
#define LED_OFF                     GPIO_PIN_SET

#define STATUS_LED_ON()             HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_ON)
#define STATUS_LED_OFF()            HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, LED_OFF)
#define STATUS_LED_TOGGLE()         HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)
#define GET_STATUS_LED()            HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)

#define FAULT_LED_ON()              HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, LED_ON)
#define FAULT_LED_OFF()             HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, LED_OFF)
#define FAULT_LED_TOGGLE()          HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin)
#define GET_FAULT_LED()             HAL_GPIO_ReadPin(FAULT_LED_GPIO_Port, FAULT_LED_Pin)

#define COOLER_ON()                 HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET)
#define COOLER_OFF()                HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET)
#define RELAY_ON()                  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET)
#define RELAY_OFF()                 HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET)

#define L6470_EMERGENCY_STOP        HAL_GPIO_WritePin(L6470_RST_GPIO_Port, L6470_RST_Pin, GPIO_PIN_RESET); while(1);

#define SLAVE_RS485_SEND_MODE       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, TRUE);
#define SLAVE_RS485_RECEIVE_MODE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, FALSE);

#define M25AA_CS_LOW()              HAL_GPIO_WritePin(M25AA_SS_GPIO_Port, M25AA_SS_Pin, GPIO_PIN_RESET);
#define M25AA_CS_HIGH()             HAL_GPIO_WritePin(M25AA_SS_GPIO_Port, M25AA_SS_Pin, GPIO_PIN_SET);


extern ADC_HandleTypeDef    hadc;
extern DAC_HandleTypeDef    hdac1;
extern I2C_HandleTypeDef    hi2c1;
extern IWDG_HandleTypeDef   hiwdg;
extern SPI_HandleTypeDef    hspi1;
extern TIM_HandleTypeDef    htim6;        //  Modbus Timer
extern TIM_HandleTypeDef    htim14;       //  Delay Timer
extern TIM_HandleTypeDef    htim15;       //
extern TIM_HandleTypeDef    htim16;       //  Beeper Timer
extern TIM_HandleTypeDef    htim17;       //  PWM timer
extern UART_HandleTypeDef   huart1;

/*  UART  */
extern UART_HandleTypeDef*  ports[];


void BSP_HW_Init(void);
void BSP_Delay(uint32_t delay);

HAL_StatusTypeDef           BSP_UartConfig( uint8_t ucPort, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity );
HAL_StatusTypeDef           BSP_UartStart( uint8_t ucPORT );
HAL_StatusTypeDef           BSP_UartStop( uint8_t ucPORT );
void                        BSP_UART_IRQ_Handler(USART_TypeDef * usart);
__INLINE static void        BSP_UartRxEnable(uint8_t ucPORT);
__INLINE static void        BSP_UartRxDisable(uint8_t ucPORT);
__INLINE static void        BSP_UartTxEnable(uint8_t ucPORT);
__INLINE static void        BSP_UartTxDisable(uint8_t ucPORT);
__INLINE static uint8_t     BSP_GetReceivedByte(uint8_t ucPORT);
__INLINE static void        BSP_PutByteToUart(uint8_t ucPORT, uint8_t ucByte);
void                        BSP_UART_IRQ_Handler(USART_TypeDef * usart);

HAL_StatusTypeDef           CheckBaudrateValue(uint32_t baudrate);
HAL_StatusTypeDef           CheckBaudrateIndex( uint8_t idx );

uint8_t                     GetIndexByBaudrate( uint32_t baudrate );    // grazina bodreito indeksa
uint32_t                    GetBaudrateByIndex( uint8_t idx );  // grazina bodreita pagal jo indeksa

uint8_t                     GetCurrentBaudrateIndex( void );    // grazina aktyvaus bodreito indeksa
uint8_t                     GetCurrentParity( void );           // grazina aktyvu parity reiksme
uint8_t                     GetCurrentStopBits( void );
uint8_t                     GetCurrentDataBits( void );


/*  TIMERS  */
void                        BSP_MbPortTimerInit(uint16_t period);
__INLINE static void        BSP_MbPortTimerEnable(void);
__INLINE static void        BSP_MbPortTimerDisable(void);
void                        TIM6_DAC1_IRQHandler(void);

void                        BSP_PwmTimerInit(void);
void                        BSP_SetAnalogOut(float volts);

void                        BSP_SetDAC_Value(float dac);

void                        BSP_ReadDipSwitch(void);



HAL_StatusTypeDef BSP_SpiTx(uint8_t* pData, uint8_t len);
HAL_StatusTypeDef BSP_SpiRx(uint8_t* pData, uint8_t len);
HAL_StatusTypeDef BSP_SpiRxTx(uint8_t* pData, uint8_t len);



/* UART INLINE FUNCTIONS */
__INLINE void BSP_UartRxEnable(uint8_t ucPORT){
    __HAL_UART_ENABLE_IT( ports[ucPORT], UART_IT_RXNE );
}

/*  */
__INLINE void BSP_UartRxDisable(uint8_t ucPORT){
    __HAL_UART_DISABLE_IT( ports[ucPORT], UART_IT_RXNE );
}

/*  */
__INLINE void BSP_UartTxEnable(uint8_t ucPORT){
    __HAL_UART_ENABLE_IT( ports[ucPORT], UART_IT_TXE );
}

/*  */
__INLINE static void BSP_UartTxDisable(uint8_t ucPORT){
    __HAL_UART_DISABLE_IT( ports[ucPORT], UART_IT_TXE );
}


/*  */
__INLINE uint8_t BSP_GetReceivedByte(uint8_t ucPORT){
    return ports[ucPORT]->Instance->RDR;
}


/*  */
__INLINE void BSP_PutByteToUart(uint8_t ucPORT, uint8_t ucByte){
    ports[ucPORT]->Instance->TDR = ucByte;
}



/* MODBUS TIMER INLINE FUNCTIONS */
__INLINE void BSP_MbPortTimerEnable(void) {

    __HAL_TIM_SET_COUNTER(&htim6, 0);

    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

    if ( HAL_TIM_Base_Start(&htim6) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/*  */
__INLINE void BSP_MbPortTimerDisable(void) {

    __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);

    if ( HAL_TIM_Base_Stop(&htim6) != HAL_OK ) {
        _Error_Handler(__FILE__, __LINE__);
    }
}




#endif /* BOARD_H_INCLUDED */
