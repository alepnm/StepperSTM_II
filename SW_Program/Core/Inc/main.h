/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define USE_HC598 1

#define RELAY_Pin GPIO_PIN_13
#define RELAY_GPIO_Port GPIOC
#define DI3_Pin GPIO_PIN_14
#define DI3_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOA
#define SPEED_CONTROL_Pin GPIO_PIN_1
#define SPEED_CONTROL_GPIO_Port GPIOA
#define HALL_S_Pin GPIO_PIN_2
#define HALL_S_GPIO_Port GPIOA
#define HALL_S_EXTI_IRQn EXTI2_3_IRQn
#define STCK_Pin GPIO_PIN_3
#define STCK_GPIO_Port GPIOA
#define DI0_Pin GPIO_PIN_0
#define DI0_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_1
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_2
#define DI2_GPIO_Port GPIOB
#define L6470_RST_Pin GPIO_PIN_10
#define L6470_RST_GPIO_Port GPIOB
#define L6470_SS_Pin GPIO_PIN_11
#define L6470_SS_GPIO_Port GPIOB
#define M25AA_SS_Pin GPIO_PIN_12
#define M25AA_SS_GPIO_Port GPIOB
#define HC165_SS_Pin GPIO_PIN_13
#define HC165_SS_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_14
#define STATUS_LED_GPIO_Port GPIOB
#define FAULT_LED_Pin GPIO_PIN_15
#define FAULT_LED_GPIO_Port GPIOB
#define HCCTRL_Pin GPIO_PIN_11
#define HCCTRL_GPIO_Port GPIOA
#define HC165_LATCH_Pin GPIO_PIN_6
#define HC165_LATCH_GPIO_Port GPIOF
#define COOLER_Pin GPIO_PIN_7
#define COOLER_GPIO_Port GPIOF
#define BEEPER_Pin GPIO_PIN_8
#define BEEPER_GPIO_Port GPIOB
#define PWM_Pin GPIO_PIN_9
#define PWM_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define FLT_SW_MODBUS		    ( 0x0008U ) /* 3 bitas modbus steko klaida */
#define FLT_HW_HS			    ( 0x0010U ) /* 4 bitas  holo daviklio klaida */
#define FLT_HW_VBUS             ( 0x0020U ) /* 5 bitas  nera draiverio maitinimo VBUS */
#define FLT_HW_VBUS_LOW         ( 0x0040U ) /* 6 bitas  VBUS reiksme uz diapazono ribu */
#define FLT_HW_VBUS_HIGH        ( 0x0080U ) /* 7 bitas VBUS reiksme uz diapazono ribu */
//#define FLT_HW_OVERHEAT         ( 0x0100U ) /* 8 bitas draiverio perkaitimas. Signalas imamas is isorinio termodaviklio. Bus realizuota ateity */
#define FLT_HW_ULVO             ( 0x0200U ) /* 9 bitas zema draiverio itampa */
#define FLT_HW_OCD              ( 0x0400U ) /* 10 bitas overcurrent */
#define FLT_HW_TH_WRN           ( 0x0800U ) /* 11 bitas darbas uzblokuotas po perkaitimo */
#define FLT_HW_TH_SHUTDOWN      ( 0x1000U ) /* 11 bitas darbas uzblokuotas po perkaitimo. Aktyvuojamas, kai gaunama is draiverio, numetam praejus tam tikra laika (OverheatStopTimer) */
#define FLT_HW_MOTOR            ( 0x2000U ) /* 12 bitas variklio klaida - motoras nesisuka */


typedef enum {RES_OK = 0, RES_ERROR} RESULT_TypeDef;


typedef struct{

    //SMC_ModeTypeDef 	                ControlMode;                // DIP switch nustatomu opciju reiksme : valdymo rezimas
    //SMC_StateTypeDef                    SMC_State;                  //

    struct{
        uint8_t*                		pId;                        // valdiklio UID (25AA048)
        uint8_t*                		pName;                      // valdiklio pavadinimas
        uint8_t*                		pFWVersion;                   // SW versija
        uint8_t*                        pHWVersion;
        uint8_t*                        pProdCode;
    }StrData;

    struct{
        //const struct MotorParamSet*  	pCurrentMotorPreset;        // pointeris i naudojamo variklio parametru preseta
        uint16_t                        Status;                     // variklio busena, nuskaityta is draiverio L6470 per spi ( registras STATUS bitai 5-6 (MOT_STATUS) )
        uint16_t                     	RotSpeedSetting;            // nustatytas sukimosi greitis Normal rezime
        uint8_t                     	RotDirSetting;              // nustatyta sukimosi kriptys Normal rezime
    }MotorData;

    struct{
        uint8_t 			    		Data;                       // is DIP switch nuskaityta reiksme
        struct{
            uint8_t 	        		HallSensor;                 // holo daviklis yra/nera
            uint8_t         		    Scrolling;                  // Scroll rezimas naudojamas/nenaudojamas
            uint8_t 		    		MotorType;                  // naudojamo variklio tipas (0-7)
        }Option;
    }DipSwitch;

    struct{
        uint16_t                        Vbus;                       // VBUS ADC reiksme
        uint16_t                        SpReq;                      // SP_REQ ADC reiksme
        uint16_t                        McuTemp;                    // MCU temperatusa ADC reiksme
    }ADC_Vals;

}SmcHandle_TypeDef;

extern SmcHandle_TypeDef SMC_Control;

/* UART */
//#pragma anon_unions
/* @todo (demo#1#): Apgalvoti modbus duomenu struktura! */
#pragma pack(push,1)
typedef struct {

    uint8_t         Uart;

    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }MbAddr;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;      // aktyvi reiksme
    }Baudrate;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }Parity;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }StopBits;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }DataBits;
} MbPortParams_TypeDef;
#pragma pack(pop)

extern MbPortParams_TypeDef MbPortParams;


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
