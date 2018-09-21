#include "fsm.h"
#include "defs.h"
#include "board.h"

#include "fsm_init.c"
#include "fsm_stop.c"
#include "fsm_normal.c"
#include "fsm_scroll.c"
#include "fsm_fault.c"
#include "fsm_test.c"
#include "fsm_config.c"


static void( *fsm_state_tbl[] )() = { FSM_StateInitHandler, FSM_StateStopHandler, FSM_StateNormalHandler, FSM_StateFaultHandler, FSM_StateScrollHandler, FSM_StateConfigHandler, FSM_StateTestHandler };




/*  */
void FSM_Manager(void){




    if(SMC_Control.SMC_State != FSM_STATE_INIT){

        /* skaitom L6470 status registra */
        SMC_Control.MotorData.Status = L6470_getStatus();

        SystemDataUpdate();
        MbDataUpdate();
    }



    /* vykdom steita */
    fsm_state_tbl[SMC_Control.SMC_State]();
}

