

/*  */
static void FSM_StateInitHandler(void){




    SystemDataInit();
    MbDataInit();

//    if(MbPortParams.ModbusActive == false){
//        SET_BIT( usRegInputBuf[IR_FAULT_CODE], FLT_SW_MODBUS );
//        SMC_Control.SMC_State = FSM_STATE_FAULT;
//    }else{
//        SMC_Control.SMC_State = FSM_STATE_STOP;
//    }
}
