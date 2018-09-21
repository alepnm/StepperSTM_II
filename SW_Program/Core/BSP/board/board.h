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
#define GET_STATUS_LED_STATE()      HAL_GPIO_ReadPin(STATUS_LED_GPIO_Port, STATUS_LED_Pin)

#define FAULT_LED_ON()              HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, LED_ON)
#define FAULT_LED_OFF()             HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, LED_OFF)
#define FAULT_LED_TOGGLE()          HAL_GPIO_TogglePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin)
#define GET_FAULT_LED_STATE()       HAL_GPIO_ReadPin(FAULT_LED_GPIO_Port, FAULT_LED_Pin)

#define POWER_ON()                  HAL_GPIO_WritePin(PWRON_GPIO_Port, PWRON_Pin, GPIO_PIN_SET)
#define POWER_OFF()                 HAL_GPIO_WritePin(PWRON_GPIO_Port, PWRON_Pin, GPIO_PIN_RESET)

#define COOLER_ON()                 HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET)
#define COOLER_OFF()                HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET)
#define RELAY_ON()                  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET)
#define RELAY_OFF()                 HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET)

//#define SLAVE_RS485_SEND_MODE       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, TRUE)
//#define SLAVE_RS485_RECEIVE_MODE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, FALSE)

#define M25AA_CS_LOW()              HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_RESET)
#define M25AA_CS_HIGH()             HAL_GPIO_WritePin(M25AA_CS_GPIO_Port, M25AA_CS_Pin, GPIO_PIN_SET)

#define HC598_CS_LOW()              HAL_GPIO_WritePin(HC598_CS_GPIO_Port, HC598_CS_Pin, GPIO_PIN_RESET)
#define HC598_CS_HIGH()             HAL_GPIO_WritePin(HC598_CS_GPIO_Port, HC598_CS_Pin, GPIO_PIN_SET)
#define HC598_LAT_LOW()             HAL_GPIO_WritePin(HC598_LAT_GPIO_Port, HC598_LAT_Pin, GPIO_PIN_RESET)
#define HC598_LAT_HIGH()            HAL_GPIO_WritePin(HC598_LAT_GPIO_Port, HC598_LAT_Pin, GPIO_PIN_SET)
#define HC598_CTRL_LOW()            HAL_GPIO_WritePin(HC598_CTRL_GPIO_Port, HC598_CTRL_Pin, GPIO_PIN_RESET)
#define HC598_CTRL_HIGH()           HAL_GPIO_WritePin(HC598_CTRL_GPIO_Port, HC598_CTRL_Pin, GPIO_PIN_SET)


#define L6470_CS_LOW()              HAL_GPIO_WritePin(L6470_CS_GPIO_Port, L6470_CS_Pin, GPIO_PIN_RESET)
#define L6470_CS_HIGH()             HAL_GPIO_WritePin(L6470_CS_GPIO_Port, L6470_CS_Pin, GPIO_PIN_SET)
#define L6470_RST_LOW()             HAL_GPIO_WritePin(L6470_RST_GPIO_Port, L6470_RST_Pin, GPIO_PIN_RESET)
#define L6470_RST_HIGH()            HAL_GPIO_WritePin(L6470_RST_GPIO_Port, L6470_RST_Pin, GPIO_PIN_SET)
#define L6470_EMERGENCY_STOP()      L6470_RST_LOW(); while(1);

#define DI0_STATE()                 HAL_GPIO_ReadPin(DI0_GPIO_Port, DI0_Pin)
#define DI1_STATE()                 HAL_GPIO_ReadPin(DI1_GPIO_Port, DI1_Pin)
#define DI2_STATE()                 HAL_GPIO_ReadPin(DI2_GPIO_Port, DI2_Pin)
#define DI3_STATE()                 HAL_GPIO_ReadPin(DI3_GPIO_Port, DI3_Pin)

#define READ_HALL_SENSOR_INPUT()    HAL_GPIO_ReadPin(HALL_S_GPIO_Port, HALL_S_Pin)

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


void                        BSP_HW_Init(void);
__INLINE static void        BSP_Delay(uint32_t delay);
__INLINE static uint32_t    BSP_GetTick(void);

HAL_StatusTypeDef           BSP_UartConfig( uint8_t ucPort, uint32_t ulBaudRate, uint8_t ucDataBits, uint8_t eParity );
HAL_StatusTypeDef           BSP_UartStart( uint8_t ucPORT );
HAL_StatusTypeDef           BSP_UartStop( uint8_t ucPORT );
__INLINE static void        BSP_UartRxEnable(uint8_t ucPORT);
__INLINE static void        BSP_UartRxDisable(uint8_t ucPORT);
__INLINE static void        BSP_UartTxEnable(uint8_t ucPORT);
__INLINE static void        BSP_UartTxDisable(uint8_t ucPORT);
__INLINE static uint8_t     BSP_GetReceivedByte(uint8_t ucPORT);
__INLINE static void        BSP_PutByteToUart(uint8_t ucPORT, uint8_t ucByte);
void                        BSP_UART_IRQ_Handler(USART_TypeDef * usart);

/*  TIMERS  */
void                        BSP_MbPortTimerInit(uint16_t period);
void                        BSP_MbPortTimerEnable(void);
void                        BSP_MbPortTimerDisable(void);
void                        TIM6_DAC1_IRQHandler(void);

void                        BSP_PwmTimerInit(void);

void                        BSP_SoundTimerInit(void);
void                        BSP_SoundTimerStart(void);
void                        BSP_SoundTimerStop(void);

void                        BSP_ReadDipSwitch(void);
uint16_t                    BSP_GetAdcValue(uint32_t channel);

void                        BSP_SetAnalogOut(float volts);
void                        BSP_SetDAC_Value(float dac);

void                        BSP_SystemReset(void);


HAL_StatusTypeDef           BSP_L6470_SPI_TransmitReceive(uint8_t* pDataTx, uint8_t* pDataRx, uint8_t len);

HAL_StatusTypeDef           BSP_M25AAxx_SPI_Transmit(uint8_t* pData, uint8_t len);
HAL_StatusTypeDef           BSP_M25AAxx_SPI_Receive(uint8_t* pData, uint8_t len);



HAL_StatusTypeDef           BSP_IIC_IsReady(uint8_t base_addr);
HAL_StatusTypeDef           BSP_IIC_Read(uint16_t eeaddr, uint16_t mem_addr, uint8_t* pData, uint16_t len);
HAL_StatusTypeDef           BSP_IIC_Write(uint16_t eeaddr, uint16_t mem_addr, uint8_t* pData, uint16_t len);



/* INLINE funkcijos */

/*  */
__INLINE void BSP_Delay(uint32_t delay) {
    HAL_Delay(delay);
}

/*  */
__INLINE uint32_t BSP_GetTick(void) {
    return HAL_GetTick();
}



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


#endif /* BOARD_H_INCLUDED */
