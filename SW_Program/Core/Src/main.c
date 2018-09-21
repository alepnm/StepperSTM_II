
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "common.h"
#include "fsm.h"
#include "mb.h"
#include "user_mb_app.h"
#include "M25AAxx.h"
#include "l6470.h"
#include "sound.h"

#define UNIT_GROUP          0x04
#define UNIT_SUBGROUP       0x01
#define UNIT_NAME           "STP"
#define UNIT_FW_VERSION     "20"
#define UNIT_HW_VERSION     "30"
#define UNIT_PROD_CODE      "GRG060"

#define TEMP110_CAL_ADDR                ( (uint16_t*) ((uint32_t) 0x1FFFF7C2) )
#define TEMP30_CAL_ADDR                 ( (uint16_t*) ((uint32_t) 0x1FFFF7B8) )
#define VDD_CALIB                       ( (uint16_t) (330) )
#define VDD_APPLI                       ( (uint16_t) (300) )
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SmcHandle_TypeDef SMC_Control;

MbPortParams_TypeDef MbPortParams = {
    .Uart = MB_PORT_DEF,
    .ModbusActive = false,
    .MbAddr = { .pmbus = &usRegHoldingBuf[HR_MBADDR], .cvalue = MBADDR_DEF},
    .Baudrate = { .pmbus = &usRegHoldingBuf[HR_MBBAUDRATE], .cvalue = MBBAURATE_DEF },
    .Parity = { .pmbus = &usRegHoldingBuf[HR_MBPARITY], .cvalue = MBPARITY_DEF },
    .StopBits = { .pmbus = &usRegHoldingBuf[HR_MBSTOPBITS], .cvalue = MBSTOPBITS_DEF },
    .DataBits  = { .pmbus = NULL, .cvalue = MBWORDLENGHT_DEF }
};

static uint32_t timestamp = 0;
static uint32_t wtime = 0;

static uint8_t uStepRegisterValue = 0;
static uint32_t OverheatStopTimer;

static FlagStatus CoolerOnBit = RESET;
static FlagStatus SaveWTimeFlag = RESET;
static FlagStatus SystemNeedReInit = RESET;
static FlagStatus SystemNeedReBoot = RESET;
static FlagStatus ReadAnalogsFlag = RESET;

static const uint32_t baudrates[6u] = {2400u, 4800u, 9600u, 19200u, 38400u, 57600u};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);




/* USER CODE BEGIN PFP */
void                SystemDataInit(void);
void                MbDataInit(void);
void                ReadDipSwitch(void);
void                EEDataRestore(void);
void                SystemDataUpdate(void);
void                MbDataUpdate(void);
void                ReadAnalogInputs(void);
void                MotorConfig( const MotorParamSet* preset );
void                CoolerController(void);
void                CoolerOnByTime(uint8_t sec);
void                RelayController(void);
void                LedsController(void);
void                SystemReset(void);


HAL_StatusTypeDef   CheckBaudrateValue(uint32_t baudrate);
HAL_StatusTypeDef   CheckBaudrateIndex( uint8_t idx );

uint8_t             GetIndexByBaudrate( uint32_t baudrate );    // grazina bodreito indeksa
uint32_t            GetBaudrateByIndex( uint8_t idx );  // grazina bodreita pagal jo indeksa

uint8_t             GetCurrentBaudrateIndex( void );    // grazina aktyvaus bodreito indeksa
uint8_t             GetCurrentParity( void );           // grazina aktyvu parity reiksme
uint8_t             GetCurrentStopBits( void );
uint8_t             GetCurrentDataBits( void );

uint8_t             InverseBits(uint8_t data);

/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    static uint32_t delay = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_DAC1_Init();
  MX_I2C1_Init();
  //MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

    wtime = GetWTime();

    BSP_HW_Init();

    SMC_Control.SMC_State = FSM_STATE_INIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if(delay < timestamp){

            /* resetinam watchdog'a */
            if( xMbGetCoil( CO_WDT_FUNC ) != RESET ){
                (void)HAL_IWDG_Refresh(&hiwdg);
            }

            delay = timestamp + 20;


            SystemDataUpdate();

            MbDataUpdate();

            LedsController();

            FSM_Manager();
      }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    (void)eMBPoll();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim15);

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 24000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim17);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY_Pin|PWRON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L6470_RST_Pin|STATUS_LED_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L6470_CS_Pin|M25AA_CS_Pin|HC598_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HC598_CTRL_GPIO_Port, HC598_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, HC598_LAT_Pin|COOLER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_Pin PWRON_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|PWRON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DI3_Pin */
  GPIO_InitStruct.Pin = DI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DI3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_S_Pin */
  GPIO_InitStruct.Pin = HALL_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_S_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI0_Pin DI1_Pin DI2_Pin */
  GPIO_InitStruct.Pin = DI0_Pin|DI1_Pin|DI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L6470_RST_Pin STATUS_LED_Pin FAULT_LED_Pin */
  GPIO_InitStruct.Pin = L6470_RST_Pin|STATUS_LED_Pin|FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L6470_CS_Pin M25AA_CS_Pin HC598_CS_Pin */
  GPIO_InitStruct.Pin = L6470_CS_Pin|M25AA_CS_Pin|HC598_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : HC598_CTRL_Pin */
  GPIO_InitStruct.Pin = HC598_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HC598_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HC598_LAT_Pin COOLER_Pin */
  GPIO_InitStruct.Pin = HC598_LAT_Pin|COOLER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}

/* USER CODE BEGIN 4 */

/*  */
void SystemDataInit(void){

    uint8_t i = 0;

    ReadDipSwitch();
    UserTimersInit();
    SoundInit( true );

    EEDataRestore();

    SMC_Control.StrData.pHWVersion = ucSlaveIdBuf + 3;
    SMC_Control.StrData.pFWVersion = ucSlaveIdBuf + 6;
    SMC_Control.StrData.pId = ucSlaveIdBuf + 12;
    SMC_Control.StrData.pName = ucSlaveIdBuf + 19;
    SMC_Control.StrData.pProdCode = ucSlaveIdBuf + 23;


    ucSlaveIdBuf[0] =   UNIT_GROUP;
    ucSlaveIdBuf[1] =   UNIT_SUBGROUP;
    ucSlaveIdBuf[2] =   'H';
    ucSlaveIdBuf[5] =   'F';
    ucSlaveIdBuf[8] =   'S';
    ucSlaveIdBuf[11] =  'I';
    ucSlaveIdBuf[18] =  'e';

    memcpy(SMC_Control.StrData.pFWVersion, UNIT_FW_VERSION, 2);
    memcpy(SMC_Control.StrData.pHWVersion, UNIT_HW_VERSION, 2);
    memcpy(SMC_Control.StrData.pName, UNIT_NAME, 3);
    memcpy(SMC_Control.StrData.pProdCode, UNIT_PROD_CODE, 7);


    do{
        *(SMC_Control.StrData.pId+i) = M25AAxx.UidBuffer[i];
    }while(++i < M25AAxx_UID_BUFFER_SIZE);


    MbPortParams.Uart = 0;
    MbPortParams.MbAddr.cvalue = usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.cvalue = usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.cvalue = usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.cvalue = MBWORDLENGHT_DEF;
    MbPortParams.StopBits.cvalue = usRegHoldingBuf[HR_MBSTOPBITS];

    SMC_Control.MotorData.pCurrentMotorPreset = GetPresetByID(SMC_Control.DipSwitch.Option.MotorType);
}

/*  */
void MbDataInit(void){

    __enter_critical();

    *MbPortParams.MbAddr.pmbus = MbPortParams.MbAddr.cvalue;
    *MbPortParams.Baudrate.pmbus = MbPortParams.Baudrate.cvalue;
    *MbPortParams.Parity.pmbus = MbPortParams.Parity.cvalue;
    *MbPortParams.DataBits.pmbus = MbPortParams.DataBits.cvalue;
    *MbPortParams.StopBits.pmbus = MbPortParams.StopBits.cvalue;


#ifdef MODBUS_ENABLE
    if( eMBInit( MB_RTU, (UCHAR)(*MbPortParams.MbAddr.pmbus), MbPortParams.Uart, (ULONG)( GetBaudrateByIndex(*MbPortParams.Baudrate.pmbus) ), (eMBParity)(*MbPortParams.Parity.pmbus) ) == MB_ENOERR ){
        if( eMBEnable() == MB_ENOERR ){
            if( eMBSetSlaveID( 123, TRUE, ucSlaveIdBuf, (MB_FUNC_OTHER_REP_SLAVEID_BUF - 4) ) == MB_ENOERR ){
                MbPortParams.ModbusActive = true;
            }
        }
    }
#endif

    if(MbPortParams.ModbusActive == false){
        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_SW_MODBUS );
        SMC_Control.SMC_State = FSM_STATE_FAULT;
    }else{
        SMC_Control.SMC_State = FSM_STATE_STOP;
    }

    __exit_critical();
}

/*  */
void EEDataRestore(void) {

    /* jai EEPROM ne inicializuotas, inicializuojam ji */
    if( M25AAxx_ReadByte( EEADDR_INIT_BYTE ) != EE_INIT_BYTE ) {
        M25AAxx_WriteByte( EEADR_MBADDR, MBADDR_DEF );
        M25AAxx_WriteByte( EEADDR_MBBAUDRATE, MBBAURATE_DEF );
        M25AAxx_WriteByte( EEADR_PARITY, MBPARITY_DEF );
        M25AAxx_WriteByte( EEADR_STOPBITS, MBSTOPBITS_DEF );

        M25AAxx_WriteByte( EEADR_WDT_FUNC, WDT_FUNC_DEF );

        M25AAxx_WriteByte( EEADR_MICROSTEPS, MICROSTEPS_DEF );

        M25AAxx_WriteByte( EEADR_HS_TO_VALUE, HS_TO_VALUE_DEF );

        M25AAxx_WriteByte( EEADR_SOUND_LEVEL, SOUND_LEVEL_DEF );

        M25AAxx_WriteByte( EEADR_SCROLL_RPM, SCROLL_RPM_DEF );
        M25AAxx_WriteWord( EEADR_SCROLL_OFF_CYCLE_TIME, SCROLL_OFF_CYCLE_TIME_DEF );
        M25AAxx_WriteWord( EEADR_SCROLL_ON_CYCLE_TIME, SCROLL_ON_CYCLE_TIME_DEF );
        M25AAxx_WriteByte( EEADR_SCROLL_SYNC, SCROLL_SYNC_DEF );

        M25AAxx_WriteByte( EEADR_USERSET_STEPS_PER_REV, USERSET_STEPS_PER_REV_DEF );

        M25AAxx_WriteByte( EEADR_USERSET_KVAL_RUN, USERSET_KVAL_RUN_PROC_DEF );
        M25AAxx_WriteByte( EEADR_USERSET_KVAL_ACC, USERSET_KVAL_ACC_PROC_DEF );
        M25AAxx_WriteByte( EEADR_USERSET_KVAL_DEC, USERSET_KVAL_DEC_PROC_DEF );
        M25AAxx_WriteByte( EEADR_USERSET_KVAL_HOLD, USERSET_KVAL_HOLD_PROC_DEF );

        M25AAxx_WriteWord( EEADR_USERSET_TRES_OCD, USERSET_TRES_OCD_MA_DEF );
        M25AAxx_WriteWord( EEADR_USERSET_TRES_STALL, USERSET_TRES_STALL_MA_DEF );

        M25AAxx_WriteByte( EEADR_MIN_RPM, RPM_MIN_DEF );
        M25AAxx_WriteByte( EEADR_MAX_RPM, RPM_MAX_DEF );

        M25AAxx_WriteWord( EEADR_USERSET_SPEED_ACC, USERSET_SPEED_ACC_DEF );
        M25AAxx_WriteWord( EEADR_USERSET_SPEED_DEC, USERSET_SPEED_DEC_DEF );

        M25AAxx_WriteWord( EEADR_OVH_TIMEOUT, OVH_TIMEOUT_DEF );

        M25AAxx_WriteByte( EEADR_TRANSMISSION_RATIO, TRANSMISSION_RATIO_DEF );

        M25AAxx_WriteByte( EEADDR_INIT_BYTE, EE_INIT_BYTE );
    }

    usRegHoldingBuf[HR_MBADDR] = M25AAxx_ReadByte( EEADR_MBADDR );
    usRegHoldingBuf[HR_MBBAUDRATE] = 3;//M25AAxx_ReadByte( EEADDR_MBBAUDRATE );
    usRegHoldingBuf[HR_MBPARITY] = M25AAxx_ReadByte( EEADR_PARITY );
    usRegHoldingBuf[HR_MBSTOPBITS] = M25AAxx_ReadByte( EEADR_STOPBITS );

    xMbSetCoil( CO_WDT_FUNC, ( M25AAxx_ReadByte( EEADR_WDT_FUNC) == DISABLE ) ? DISABLE : 0x01 );

    usRegHoldingBuf[HR_MICROSTEPS] = M25AAxx_ReadByte( EEADR_MICROSTEPS );

    usRegHoldingBuf[HR_HS_TO_VALUE] = M25AAxx_ReadByte( EEADR_HS_TO_VALUE );

    usRegHoldingBuf[HR_OVH_TIMEOUT] = M25AAxx_ReadWord( EEADR_OVH_TIMEOUT );

    usRegHoldingBuf[HR_SOUND_LEVEL] = M25AAxx_ReadByte( EEADR_SOUND_LEVEL );

    usRegHoldingBuf[HR_SCROLL_RPM] = M25AAxx_ReadByte( EEADR_SCROLL_RPM );
    usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] = M25AAxx_ReadWord( EEADR_SCROLL_OFF_CYCLE_TIME );
    usRegHoldingBuf[HR_SCROLL_ON_CYCLE_TIME] = M25AAxx_ReadWord( EEADR_SCROLL_ON_CYCLE_TIME );
    usRegHoldingBuf[HR_SCROLL_SYNC] = M25AAxx_ReadByte( EEADR_SCROLL_SYNC );

    usRegHoldingBuf[HR_USERSET_STEPS_PER_REV] = M25AAxx_ReadByte( EEADR_USERSET_STEPS_PER_REV );

    usRegHoldingBuf[HR_USERSET_KVAL_RUN] = M25AAxx_ReadByte( EEADR_USERSET_KVAL_RUN );
    usRegHoldingBuf[HR_USERSET_KVAL_ACC] = M25AAxx_ReadByte( EEADR_USERSET_KVAL_ACC );
    usRegHoldingBuf[HR_USERSET_KVAL_DEC] = M25AAxx_ReadByte( EEADR_USERSET_KVAL_DEC );
    usRegHoldingBuf[HR_USERSET_KVAL_HOLD] = M25AAxx_ReadByte( EEADR_USERSET_KVAL_HOLD );

    usRegHoldingBuf[HR_USERSET_TRES_OCD] = M25AAxx_ReadWord( EEADR_USERSET_TRES_OCD);
    usRegHoldingBuf[HR_USERSET_TRES_STALL] = M25AAxx_ReadWord( EEADR_USERSET_TRES_STALL);

    usRegHoldingBuf[HR_MIN_RPM] = M25AAxx_ReadByte( EEADR_MIN_RPM );
    usRegHoldingBuf[HR_MAX_RPM] = M25AAxx_ReadByte( EEADR_MAX_RPM );

    usRegHoldingBuf[HR_USERSET_SPEED_ACC] = M25AAxx_ReadWord( EEADR_USERSET_SPEED_ACC );
    usRegHoldingBuf[HR_USERSET_SPEED_DEC] = M25AAxx_ReadWord( EEADR_USERSET_SPEED_DEC );

    usRegHoldingBuf[HR_TRANSMISSION_RATIO] = M25AAxx_ReadByte( EEADR_TRANSMISSION_RATIO );

    wtime = M25AAxx_ReadDWord(EEADR_WTIME);
    SetWTime(wtime);

    usRegInputBuf[IR_WTIMEHI] = HI16(wtime);
    usRegInputBuf[IR_WTIMELO] = LO16(wtime);
}


/*  */
void SystemDataUpdate(void){

    FlagStatus port_need_update = RESET;

    __enter_critical();

    ReadDipSwitch();
    ReadAnalogInputs();

    /* chekinam, ar nepasikeite porto parametrai; jai pasikeite - chekinam reiksme ir aktyvuojam porto rekonfiguracija.
    Jai parametras neteisingas, nekeiciam ji */
    if( GetCurrentMbAddress() != usRegHoldingBuf[HR_MBADDR] ){
        if(usRegHoldingBuf[HR_MBADDR] > 247U){
            usRegHoldingBuf[HR_MBADDR] = GetCurrentMbAddress();
        }else{
            M25AAxx_WriteByte( EEADR_MBADDR, (uint8_t)usRegHoldingBuf[HR_MBADDR] );
            port_need_update = SET;
        }
    }

    if( GetCurrentBaudrateIndex() != usRegHoldingBuf[HR_MBBAUDRATE] ){
        if(usRegHoldingBuf[HR_MBBAUDRATE] > 5U){
            usRegHoldingBuf[HR_MBBAUDRATE] = GetCurrentBaudrateIndex();
        }else{
            M25AAxx_WriteByte( EEADDR_MBBAUDRATE, (uint8_t)usRegHoldingBuf[HR_MBBAUDRATE] );
            port_need_update = SET;
        }
    }

    if( GetCurrentParity() != usRegHoldingBuf[HR_MBPARITY] ){
        if(usRegHoldingBuf[HR_MBPARITY] > 2U){
            usRegHoldingBuf[HR_MBPARITY] = GetCurrentParity();
        }else{
            M25AAxx_WriteByte( EEADR_PARITY, (uint8_t)usRegHoldingBuf[HR_MBPARITY] );
            port_need_update = SET;
        }
    }

    if( GetCurrentStopBits() != usRegHoldingBuf[HR_MBSTOPBITS] ){
        if(usRegHoldingBuf[HR_MBSTOPBITS] == 0U || usRegHoldingBuf[HR_MBSTOPBITS] > 2U){
            usRegHoldingBuf[HR_MBSTOPBITS] = GetCurrentStopBits();
        }else{
            M25AAxx_WriteByte( EEADR_STOPBITS, (uint8_t)usRegHoldingBuf[HR_MBSTOPBITS] );
            port_need_update = SET;
        }
    }


    /* restartojam porta su naujais parametrais, jai reikia */
    if( port_need_update != RESET ){
        (void)eMBDisable();
        eMBInit( MB_RTU, (UCHAR)(usRegHoldingBuf[HR_MBADDR]), 0U, (ULONG)( GetBaudrateByIndex(usRegHoldingBuf[HR_MBBAUDRATE]) ), (eMBParity)(usRegHoldingBuf[HR_MBPARITY]) );
        (void)eMBEnable();
    }


    /* kas 60s saugojam wtime */
    if( SaveWTimeFlag != RESET ){
        SaveWTimeFlag = RESET;
        M25AAxx_WriteDWord( EEADR_WTIME, wtime );
    }




    /* daugiafunkcinis registras */
    switch( usRegHoldingBuf[HR_MAGIC_REG] ) {
    case 0x0000:

        /* iseinam is TESTO */
        if(SMC_Control.SMC_State == FSM_STATE_TEST) SMC_Control.SMC_State = FSM_STATE_STOP;


        break;
    case 0x16AD:    // reinicializacija defaultais
        usRegHoldingBuf[HR_MAGIC_REG] = 0x0000;
        SystemNeedReInit = SET;
        break;
    case 0x1988:    // reboot
        usRegHoldingBuf[HR_MAGIC_REG] = 0x0000;
        SystemNeedReBoot = SET;
        break;
    case 0x2A14:    // ventiliatoriaus ijungimas/isjungimas
        usRegHoldingBuf[HR_MAGIC_REG] = 0x0000;
        xMbSetCoil( CO_COOLER_ON, !xMbGetCoil( CO_COOLER_ON ) );
        break;
    case 0x2A15:    // reles ijungima/isjungimas
        usRegHoldingBuf[HR_MAGIC_REG] = 0x0000;
        xMbSetCoil( CO_RELAY_ON, !xMbGetCoil( CO_RELAY_ON ) );
        break;
    case 0x8692:
        SMC_Control.SMC_State = FSM_STATE_TEST;
        break;
    case 0xABBA:    // istrinam WTIME
        wtime = usRegHoldingBuf[HR_MAGIC_REG] = 0x0000;
        SaveWTimeFlag = SET;
        break;
    default:
        break;
    }


    /* jai reikia, inicializuojames Defaultais */
    if( SystemNeedReInit != RESET ){
        M25AAxx_WriteByte( EEADDR_INIT_BYTE, 0xFF );
        SystemNeedReInit = RESET;
        SystemNeedReBoot = SET;
    }

    /* jai reikia, persikraunam */
    if( SystemNeedReBoot != RESET ){

        HAL_Delay(100);

        SystemNeedReBoot = RESET;
        SystemReset();
    }

    __exit_critical();
}


/*  */
void MbDataUpdate(void){

    usRegInputBuf[IR_WTIMEHI] = LO16(wtime);
    usRegInputBuf[IR_WTIMELO] = HI16(wtime);

    xMbSetDInput( DI_SW1_STATE, SMC_Control.DipSwitch.Data & 0x01 );
    xMbSetDInput( DI_SW2_STATE, SMC_Control.DipSwitch.Data>>1 & 0x01 );
    xMbSetDInput( DI_SW3_STATE, SMC_Control.DipSwitch.Data>>2 & 0x01 );
    xMbSetDInput( DI_SW4_STATE, SMC_Control.DipSwitch.Data>>3 & 0x01 );
    xMbSetDInput( DI_SW5_STATE, SMC_Control.DipSwitch.Data>>4 & 0x01 );
    xMbSetDInput( DI_SW6_STATE, SMC_Control.DipSwitch.Data>>5 & 0x01 );
    xMbSetDInput( DI_SW7_STATE, SMC_Control.DipSwitch.Data>>6 & 0x01 );
    xMbSetDInput( DI_SW8_STATE, SMC_Control.DipSwitch.Data>>7 & 0x01 );


    /* skaitom skaitmeninius iejimus */
    xMbSetDInput( DI_DI0_STATE, DI0_STATE() );
    xMbSetDInput( DI_DI1_STATE, DI1_STATE() );
    xMbSetDInput( DI_DI2_STATE, DI2_STATE() );
    xMbSetDInput( DI_DI3_STATE, DI3_STATE() );

}


/*  */
void ReadDipSwitch(void) {

    static uint8_t ldipsw;

    BSP_ReadDipSwitch();

    uint8_t dipsw = SMC_Control.DipSwitch.Data^0xFF;    // invertuojam

    if( dipsw != ldipsw ) {
        SoundStart(1);
        ldipsw = dipsw;
    }

    /* jai esam STOP rezime nustatom parametrus */
    SMC_Control.DipSwitch.Option.MotorType = ( dipsw & 0x07 );
    usRegInputBuf[IR_MAX_RPM] = ( READ_BIT(dipsw, 0x01<<3) == FALSE ) ? 150 : 200;

    SMC_Control.DipSwitch.Option.Scrolling = ( dipsw>>4 & 0x01 );
    SMC_Control.DipSwitch.Option.HallSensor = ( dipsw>>5 & 0x01 );
    SMC_Control.ControlMode = ( dipsw>>6 & 0x03 );
}



/* Suvidurkintus rezultatus sudedam i tam skirtus registrus */
void ReadAnalogInputs(void) {

    static uint8_t stage = 0, n_spreq = 0, n_vbus = 0, n_itemp = 0;
    static uint32_t sum_spreq = 0, sum_vbus = 0, sum_itemp = 0;

    uint16_t adc = 0;

    __enter_critical();

    switch(stage) {
    case 0:

        if(n_vbus++ < 64) sum_vbus += BSP_GetAdcValue(ADC_CHANNEL_0);
        else {

            SMC_Control.ADC_Vals.Vbus = (uint16_t)(sum_vbus>>6);
            n_vbus = sum_vbus = 0;
            usRegInputBuf[IR_VBUS_VALUE] = SMC_Control.ADC_Vals.Vbus * 0.822;   // verciam voltais  ( formatas V*100 )
        }

        stage = 1;
        break;
    case 1:

        adc = BSP_GetAdcValue(ADC_CHANNEL_1);

        /* filtruojam triuksma ir vidurkinam ADC reiksme */
        if( SMC_Control.ADC_Vals.SpReq < adc - 20 || SMC_Control.ADC_Vals.SpReq > adc + 20 ) {

            if(n_spreq++ < 8) sum_spreq += adc;
            else {

                SMC_Control.ADC_Vals.SpReq = (uint16_t)(sum_spreq>>3);
                n_spreq = sum_spreq = 0;
                usRegInputBuf[IR_SPREQ_VALUE] = SMC_Control.ADC_Vals.SpReq * 0.235;   // verciam voltais  ( formatas V*100 )
            }
        }

        stage = 2;
        break;
    case 2:

        if(n_itemp++ < 8) sum_itemp += BSP_GetAdcValue(ADC_CHANNEL_TEMPSENSOR);
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



/* Draiverio konfiguravimas aktyviam presetui
*/
void MotorConfig( const MotorParamSet* preset ) {

    __enter_critical();

    /* L6470 registru inicializavimas */
    L6470_setMicroSteps( uStepRegisterValue );

    SetParam( REG_KVAL_RUN, PROC_8BIT(preset->Kval.RunValue) );
    SetParam( REG_KVAL_ACC, PROC_8BIT(preset->Kval.AccValue) );
    SetParam( REG_KVAL_DEC, PROC_8BIT(preset->Kval.DecValue) );
    SetParam( REG_KVAL_HOLD, PROC_8BIT(preset->Kval.HoldValue) );

    L6470_setMinSpeed( ConvertRpmToStepsPerSec(RPM_MIN_DEF) );
    L6470_setMaxSpeed( ConvertRpmToStepsPerSec(usRegInputBuf[IR_MAX_RPM]) );

    L6470_setThresholdSpeed( ConvertRpmToStepsPerSec(300) );

    L6470_setAcc( preset->Speed.Acceleration );
    L6470_setDec( preset->Speed.Deceleration );

    L6470_setOverCurrent( preset->Treshold.OcdValue );
    L6470_setStallCurrent( preset->Treshold.StallValue );

    __exit_critical();
}



/* Ausintuvo hendleris
Kuleris aktyvuojamas L6470 overheat signalu arba Modbus CO_COOLER_ON registru
T >= 20ms, main
*/
void CoolerController(void) {

    /* jai draiverio perkaitimas, aktyvuojam kuleri, jai ne - pagal COOLER Modbus bituka */
    if( READ_BIT( SMC_Control.MotorData.Status, STATUS_TH_WRN ) == RESET ) {

        /* uztaisom perkaitimo apsaugos taimeri */
        CoolerOnByTime( usRegHoldingBuf[HR_OVH_TIMEOUT] );

    } else {

        /* jai nustatytas kulerio MODBUS bitas */
        if( xMbGetCoil( CO_COOLER_ON ) != FALSE ) {
            CoolerOnBit = SET;
        } else {

            /* jai baigesi OverheatStopTimer taimerio laikas ir is draiverio negaunam Overheat alarma, numetam alarmo bita */
            if( OverheatStopTimer < timestamp ) {
                CoolerOnBit = RESET;
            }
        }
    }


    if(CoolerOnBit != RESET) COOLER_ON();
    else COOLER_OFF();
}

/*  */
void CoolerOnByTime(uint8_t sec){

    CoolerOnBit = SET;
    OverheatStopTimer = timestamp + sec * 1000;
}


/* Reles handleris
Alarm rele aktyvuojama esant kritinei klaidai arba Modbus CO_RELAY_ON registru
Kritines klaidos:
1.
2.
3.
4.

T >= 20ms, main
*/
void RelayController(void) {

    if( xMbGetCoil( CO_RELAY_ON ) != FALSE ) RELAY_ON();
    else RELAY_OFF();
}



/* Status ir Fault indikatoriu valdiklis. T = 100ms
Klaidu rodymas:
FAULT ledas sviecia pastoviai. Draiverio ledas rodo draiverio Overheat ir Overcurrent alarmus.
STATUS ledas mirksejimais rodo klaidos koda:

x1 - VBUS klaida
x2 - Hall sensor klaida
x3 - Motor klaida
x4 -

Kaip rodom kelios klaidos???
*/
void LedsController(void) {

    static uint32_t delay;
    uint16_t timeout = 0;
    static uint8_t error = 0;
    static FlagStatus led_state = 0;

    static GPIO_PinState last_hall_state = GPIO_PIN_RESET;

    if(SMC_Control.SMC_State == FSM_STATE_TEST) {

        STATUS_LED_ON();
        FAULT_LED_ON();
        return;
    }

    switch(SMC_Control.SMC_State) {

    case FSM_STATE_FAULT:

        /* FAULT ledas sviecia. Klaidos matome modbus registre */
        FAULT_LED_ON();

        /* STATUS leda panaudojam klaidos kodo parodymui */
        if(error == 0) {

            STATUS_LED_OFF();

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_SW_MODBUS ) != RESET ) error = 6;

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_HS ) != RESET ) error = 5;

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_MOTOR ) != RESET ) error = 4;

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS ) != RESET ||
                    READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_LOW ) != RESET ||
                    READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_VBUS_HIGH ) != RESET ) {

                error = 3;
            }

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_ULVO ) != RESET ) error = 2;

            if( READ_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_HW_TH_SHUTDOWN ) != RESET) error = 1;

            timeout = 1600;
        } else {

            if( delay > timestamp ) break;

            /* rodom klaida */
            switch(led_state) {
            case RESET: //dega
                STATUS_LED_ON();
                break;
            case SET: //nedega
                timeout = 300;
                STATUS_LED_OFF();
                error--;
                break;
            }

            led_state = !led_state;
        }

        delay = timestamp + timeout;

        break;
    default:

        /* gesinam FAULT leda */
        FAULT_LED_OFF();

        error = 0;

        /* STATUS ledu parodom Holo daviklio suveikima */
        if( last_hall_state != READ_HALL_SENSOR_INPUT() ) {

            STATUS_LED_ON();
            last_hall_state = READ_HALL_SENSOR_INPUT();

            delay = timestamp + 100;// <-- sumazinam uzdelsima, kad greiciau uzgestu

            break;
        }

        /* STATUS ledu rodom komtrolerio busena */
        if( ( SMC_Control.MotorData.Status & STATUS_MOT_STATUS ) != STATUS_MOT_STATUS_STOPPED ) {
            /* kai variklis sukasi, STATUS ledu sviecia nuolat */
            STATUS_LED_ON();
            delay = timestamp;
        } else {
            /* kai variklis stovi, STATUS ledu mirkciojam kas 5000 ms */
            if(delay < timestamp) {

                if( GET_STATUS_LED_STATE() == LED_OFF ) {
                    STATUS_LED_ON();
                } else {
                    STATUS_LED_OFF();
                    delay = timestamp + 5000;
                }
            }
        }

        break;
    }
}


/*   */
void SystemReset(void) {
    /* stabdom varikli */
    L6470_softStop();

    STATUS_LED_ON();
    FAULT_LED_ON();

    /* laukiam kol variklis sustos */
    while( ( SMC_Control.MotorData.Status & STATUS_MOT_STATUS ) != STATUS_MOT_STATUS_STOPPED ) {

        SMC_Control.MotorData.Status = L6470_getStatus();

        HAL_Delay(10);
    }

    HAL_Delay(300);

    BSP_SystemReset();
}



/* SYSTICK callback funkcija */
void HAL_SYSTICK_Callback(void){

    static uint8_t wr_to_eeprom_cnt = 0;
    static uint32_t time = 0u;

    SysTimeCounterUpdate();

    SoundHandler();

    timestamp = GetTimestamp();
    wtime = GetWTime();

    if( time <= timestamp ) {
        time = timestamp + 1000u;
        wtime++;

        /* kas minute EEPROMe saugojam WTIME */
        if( wr_to_eeprom_cnt++ >= 60 ){
            wr_to_eeprom_cnt = 0;
            SaveWTimeFlag = SET;
        }
    }

    ReadAnalogsFlag = SET;
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



/*  */
uint8_t InverseBits(uint8_t data) {

    int8_t i = 7;
    uint8_t j = 0, temp = 0;

    while(i >= 0) {
        temp |= ( ( data >> j++) & 1 ) << i--;
    }

    return temp;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
