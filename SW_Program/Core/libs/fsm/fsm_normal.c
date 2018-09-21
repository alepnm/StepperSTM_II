#include "user_mb_app.h"


/*  */
static void FSM_StateNormalHandler(void) {


    /* jai valdymo itampa zemesne uz 0.6V, stabdomes */
    if( usRegInputBuf[IR_SPREQ_VALUE] < 60 ) {

        L6470_softFree();

        /* laukiam kol variklis sustos, poto keiciam steita */
        if(SMC_Control.MotorData.Status == STATUS_MOT_STATUS_STOPPED) SMC_Control.SMC_State = FSM_STATE_STOP;
    }





}
