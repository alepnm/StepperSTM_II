
#include "software.h"
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


/* Global variables ---------------------------------------------------------*/
extern SmcHandle_TypeDef SMC_Control;
extern MbPortParams_TypeDef MbPortParams;
extern uint32_t timestamp, wtime;


/* Private variables ---------------------------------------------------------*/
static MotorParamSet* pUserSet;
static uint8_t uStepRegisterValue = 0;
static uint32_t OverheatStopTimer = 0, ScrollCounter = 0;;

static FlagStatus CoolerOnBit = RESET;
static FlagStatus SaveWTimeFlag = RESET;
static FlagStatus SystemNeedReInit = RESET;
static FlagStatus SystemNeedReBoot = RESET;
static FlagStatus ReadAnalogsFlag = RESET;

static const uint32_t baudrates[6u] = {2400u, 4800u, 9600u, 19200u, 38400u, 57600u};


/*  */
void SystemDataInit(void){

    uint8_t i = 0;

    EEDataRestore();

    /* Init pointers */
    MbPortParams.MbAddr.pmbus = &usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.pmbus = &usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.pmbus = &usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.pmbus = NULL;
    MbPortParams.StopBits.pmbus = &usRegHoldingBuf[HR_MBSTOPBITS];


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

#ifdef MODBUS_ENABLE
    if( eMBInit( MB_RTU, (UCHAR)(*MbPortParams.MbAddr.pmbus), MbPortParams.Uart, (ULONG)( GetBaudrateByIndex(*MbPortParams.Baudrate.pmbus) ), (eMBParity)(*MbPortParams.Parity.pmbus) ) == MB_ENOERR ){
        if( eMBEnable() == MB_ENOERR ){
            if( eMBSetSlaveID( 123, TRUE, ucSlaveIdBuf, (MB_FUNC_OTHER_REP_SLAVEID_BUF - 4) ) == MB_ENOERR ){
                MbPortParams.ModbusActive = true;
            }
        }
    }
#endif


    pUserSet = (MotorParamSet*)GetPresetByID(0x07);
    pUserSet->StepsPerRev = usRegHoldingBuf[HR_USERSET_STEPS_PER_REV];
    pUserSet->Kval.RunValue = usRegHoldingBuf[HR_USERSET_KVAL_RUN];
    pUserSet->Kval.AccValue = usRegHoldingBuf[HR_USERSET_KVAL_ACC];
    pUserSet->Kval.DecValue = usRegHoldingBuf[HR_USERSET_KVAL_DEC];
    pUserSet->Kval.HoldValue = usRegHoldingBuf[HR_USERSET_KVAL_HOLD];
    pUserSet->Treshold.OcdValue = usRegHoldingBuf[HR_USERSET_TRES_OCD];
    pUserSet->Treshold.StallValue = usRegHoldingBuf[HR_USERSET_TRES_STALL];
    pUserSet->Speed.Acceleration = usRegHoldingBuf[HR_USERSET_SPEED_ACC];
    pUserSet->Speed.Deceleration = usRegHoldingBuf[HR_USERSET_SPEED_DEC];

    SMC_Control.MotorData.pCurrentMotorPreset = GetPresetByID(SMC_Control.DipSwitch.Option.MotorType);

    MotorConfig(SMC_Control.MotorData.pCurrentMotorPreset);

    ScrollCounter = timestamp + (usRegHoldingBuf[HR_SCROLL_OFF_CYCLE_TIME] * 1000);

}

/*  */
void MbDataInit(void){

    __enter_critical();




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
uint8_t CheckBaudrateValue(uint32_t baudrate) {

    if( GetIndexByBaudrate( baudrate ) == 0xFF ) return 1;

    return 0;
}


uint8_t CheckBaudrateIndex( uint8_t idx ) {

    if( GetBaudrateByIndex( idx ) == 0xFFFFFFFF ) return 1;

    return 0;
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
