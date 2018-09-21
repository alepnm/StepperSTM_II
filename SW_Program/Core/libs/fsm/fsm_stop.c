#include "user_mb_app.h"


/*  */
static void FSM_StateStopHandler(void) {



    /* jai valdymo itampa didesne uz 1.1V, startuojam */
    if( usRegInputBuf[IR_SPREQ_VALUE] > 110 ) {

        /* pradedam sukima pries tai sukonfiguruojant draiveri aktyviam presetui */
        MotorConfig(SMC_Control.MotorData.pCurrentMotorPreset);
        L6470_run( SMC_Control.MotorData.RotDirSetting, ConvertRpmToStepsPerSec( (float)SMC_Control.MotorData.RotSpeedSetting ) );


        /* laukiam kol variklis igaus pastovu greiti, poto keiciam steita */
        if(SMC_Control.MotorData.Status == STATUS_MOT_STATUS_CONST_SPD) SMC_Control.SMC_State = FSM_STATE_NORMAL;
    }


}
