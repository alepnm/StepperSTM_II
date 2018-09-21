#ifndef DEFINES_H_INCLUDED
#define DEFINES_H_INCLUDED

#include <stdint.h>
#include "fsm.h"
#include "l6470.h"

typedef enum { RES_OK = 0, RES_ERROR, RES_BUSY, RES_TIMEOUT, RES_BAD_PARAMS } eRESULT_TypeDef;  // atitinka HAL_StatusTypeDef

#define MODBUS_ENABLE



/*   */
typedef struct{

    eMode_TypeDef 	                    ControlMode;                // DIP switch nustatomu opciju reiksme : valdymo rezimas
    eState_TypeDef                      SMC_State;                  //

    struct{
        uint8_t*                		pId;                        // valdiklio UID (25AA048)
        uint8_t*                		pName;                      // valdiklio pavadinimas
        uint8_t*                		pFWVersion;                   // SW versija
        uint8_t*                        pHWVersion;
        uint8_t*                        pProdCode;
    }StrData;

    struct{
        const MotorParamSet*  	        pCurrentMotorPreset;        // pointeris i naudojamo variklio parametru preseta
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
typedef struct {

    uint8_t         Uart;
    uint8_t         ModbusActive;

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

extern MbPortParams_TypeDef MbPortParams;


/*  */
#define     FLT_SW_MODBUS		    ( 0x0008U ) /* 3 bitas modbus steko klaida */
#define     FLT_HW_HS			    ( 0x0010U ) /* 4 bitas  holo daviklio klaida */
#define     FLT_HW_VBUS             ( 0x0020U ) /* 5 bitas  nera draiverio maitinimo VBUS */
#define     FLT_HW_VBUS_LOW         ( 0x0040U ) /* 6 bitas  VBUS reiksme uz diapazono ribu */
#define     FLT_HW_VBUS_HIGH        ( 0x0080U ) /* 7 bitas VBUS reiksme uz diapazono ribu */
//#define FLT_HW_OVERHEAT         ( 0x0100U ) /* 8 bitas draiverio perkaitimas. Signalas imamas is isorinio termodaviklio. Bus realizuota ateity */
#define     FLT_HW_ULVO             ( 0x0200U ) /* 9 bitas zema draiverio itampa */
#define     FLT_HW_OCD              ( 0x0400U ) /* 10 bitas overcurrent */
#define     FLT_HW_TH_WRN           ( 0x0800U ) /* 11 bitas darbas uzblokuotas po perkaitimo */
#define     FLT_HW_TH_SHUTDOWN      ( 0x1000U ) /* 11 bitas darbas uzblokuotas po perkaitimo. Aktyvuojamas, kai gaunama is draiverio, numetam praejus tam tikra laika (OverheatStopTimer) */
#define     FLT_HW_MOTOR            ( 0x2000U ) /* 12 bitas variklio klaida - motoras nesisuka */



/*  SYSTEM DEFAULTS */
#define     MB_PORT_DEF                 ( 0u )

#define     MBADDR_DEF                  ( 10u )     //0x0A
#define     MBPARITY_DEF                MB_PAR_NONE
#define     MBBAURATE_DEF               ( 3u )      // bodreito indeksas lenteleje ( 3->19200 )
#define     MBSTOPBITS_DEF              ( 1u )
#define     MBWORDLENGHT_DEF            ( 8u )
#define     SOUND_LEVEL_DEF             SND_OFF
#define     WDT_FUNC_DEF                DISABLE

#define     RPM_MIN_DEF                 ( 10u )     // minimalus sukimosi greitis (RPM)
#define     RPM_MAX_DEF                 ( 200u )    // maksimalus sukimosi greitis (RPM)
#define     MICROSTEPS_DEF              ( 16u )     // mikrostepu
#define     MIN_TRES_OCD_MA_DEF         ( 500u )    // minimali sroves reiksme, mA
#define     MAX_TRES_OCD_MA_DEF         ( 4000u )   // maksimali sroves reiksme, mA
#define     MAX_KVAL_VALUE_DEF          ( 60u )
#define     MAX_KVAL_HOLD_VALUE_DEF     ( 20u )
#define     HS_TO_VALUE_DEF             ( 10u )     // holo daviklio taimaut (sekundes)
#define     TRANSMISSION_RATIO_DEF      ( 30u )     // variklio ir rotoriaus diametru santykis
#define     OVH_TIMEOUT_DEF             ( 15u )     // sekundes

#define     USERSET_STEPS_PER_REV_DEF   ( 200u )    //
#define     USERSET_KVAL_RUN_PROC_DEF   ( 24u )     // %
#define     USERSET_KVAL_ACC_PROC_DEF   ( 20u )     // %
#define     USERSET_KVAL_DEC_PROC_DEF   ( 16u )     // %
#define     USERSET_KVAL_HOLD_PROC_DEF  ( 5u )      // %
#define     USERSET_TRES_OCD_MA_DEF     ( 3000u )   // mA
#define     USERSET_TRES_STALL_MA_DEF   ( 2000u )   // mA
#define     USERSET_SPEED_ACC_DEF       ( 100u )    // steps/s^2
#define     USERSET_SPEED_DEC_DEF       ( 100u )    // steps/s^2

#define     SCROLL_RPM_DEF              ( 20u )     // RPM
#define     SCROLL_OFF_CYCLE_TIME_DEF   ( 300u )    // sekundes
#define     SCROLL_ON_CYCLE_TIME_DEF    ( 30u )     // sekundes
#define     SCROLL_SYNC_DEF             ( 0u )


/* kiti defainai */
#define		TESTMODE_KEY				( 0x4949 )
#define     SERVICEMODE_KEY             ( 0x26AA )



/* EEPROM adresai */
#define EEADDR_BASE                     0
#define EEADDR_INIT_BYTE                EEADDR_BASE+3
#define EEADR_MBADDR                    EEADDR_BASE+5                   // byte
#define EEADDR_MBBAUDRATE               EEADR_MBADDR+1                  // byte
#define EEADR_PARITY                    EEADDR_MBBAUDRATE+1             // byte
#define EEADR_STOPBITS                  EEADR_PARITY+1                  // byte
#define EEADR_SCROLL_RPM                EEADR_STOPBITS+1                // byte
#define EEADR_HS_TO_VALUE               EEADR_SCROLL_RPM+1              // byte
#define EEADR_SCROLL_OFF_CYCLE_TIME     EEADR_HS_TO_VALUE+1             // word
#define EEADR_SCROLL_ON_CYCLE_TIME      EEADR_SCROLL_OFF_CYCLE_TIME+2   // word
#define EEADR_SCROLL_SYNC               EEADR_SCROLL_ON_CYCLE_TIME+2    // byte
#define EEADR_MICROSTEPS                EEADR_SCROLL_SYNC+1             // byte
#define EEADR_USERSET_STEPS_PER_REV     EEADR_MICROSTEPS+1              // byte
#define EEADR_USERSET_KVAL_RUN          EEADR_USERSET_STEPS_PER_REV+1   // byte
#define EEADR_USERSET_KVAL_ACC          EEADR_USERSET_KVAL_RUN+1        // byte
#define EEADR_USERSET_KVAL_DEC          EEADR_USERSET_KVAL_ACC+1        // byte
#define EEADR_USERSET_KVAL_HOLD         EEADR_USERSET_KVAL_DEC+1        // byte
#define EEADR_USERSET_TRES_OCD          EEADR_USERSET_KVAL_HOLD+1       // word
#define EEADR_USERSET_TRES_STALL        EEADR_USERSET_TRES_OCD+2        // word
#define EEADR_MIN_RPM                   EEADR_USERSET_TRES_STALL+2      // byte
#define EEADR_MAX_RPM                   EEADR_MIN_RPM+1                 // byte
#define EEADR_USERSET_SPEED_ACC         EEADR_MAX_RPM+1                 // word
#define EEADR_USERSET_SPEED_DEC         EEADR_USERSET_SPEED_ACC+2       // word
#define EEADR_OVH_TIMEOUT               EEADR_USERSET_SPEED_DEC+2       // word
#define EEADR_SOUND_LEVEL               EEADR_OVH_TIMEOUT+2             // byte
#define EEADR_WTIME                     EEADR_SOUND_LEVEL+1             // dword
#define EEADR_TRANSMISSION_RATIO        EEADR_WTIME+4                   // byte
#define EEADR_WDT_FUNC                  EEADR_TRANSMISSION_RATIO+1      // byte



#endif /* DEFINES_H_INCLUDED */
