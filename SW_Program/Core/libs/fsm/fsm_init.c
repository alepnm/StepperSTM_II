

/*  */
static void FSM_StateInitHandler(void){

    SystemDataInit();
    MbDataInit();

    SMC_Control.SMC_State = FSM_STATE_STOP;
}
