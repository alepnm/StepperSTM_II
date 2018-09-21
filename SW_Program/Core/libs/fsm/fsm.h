#ifndef FSM_H_INCLUDED
#define FSM_H_INCLUDED

#include "common.h"
#include "l6470.h"


typedef enum { FSM_MODE_MANUAL = 0, FSM_MODE_MODBUS, FSM_MODE_STEPCLOCK, FSM_MODE_TESTMODE } eMode_TypeDef;
typedef enum {  FSM_STATE_INIT = 0,
                FSM_STATE_STOP,
                FSM_STATE_NORMAL,
                FSM_STATE_FAULT,
                FSM_STATE_SCROLLING,
                FSM_STATE_CONFIG,
                FSM_STATE_TEST
} eState_TypeDef;



extern void                SystemDataInit(void);
extern void                MbDataInit(void);
extern void                SystemDataUpdate(void);
extern void                MbDataUpdate(void);
extern void                MotorConfig( const MotorParamSet* preset );



/* Global functions prototypes -----------------------------------------------*/
void FSM_Manager(void);

#endif /* FSM_H_INCLUDED */
