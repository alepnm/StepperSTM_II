/*systick.c

Sisteminio laiko ir vartotoju taimeriu modulis. Vartotojo taimeriai 16bit.
*/


#include "board.h"
#include "systick.h"


#define USER_TIMERS_QUANT   3


/*
#if(USER_TIMERS_QUANT > 0)
    struct _user_timer{
        uint16_t            period;
        uint16_t            counter;
        FunctionalState     is_count;
        FlagStatus          flag;
    };
#endif

struct _sys_timer{
    uint32_t            CurrentTimestamp;
    uint32_t            WTime_sec;
#if(USER_TIMERS_QUANT > 0)
    struct _user_timer  user_timers[USER_TIMERS_QUANT];
#endif
}SysTimers;
*/



#if(USER_TIMERS_QUANT > 0)
struct _user_timer {
    FunctionalState     is_count;
    uint16_t            period;
    FlagStatus          flag;
    uint16_t            counter;
};
#endif

static struct _sys_timer {
#if(USER_TIMERS_QUANT > 0)
    struct _user_timer  user_timers[USER_TIMERS_QUANT];
#endif
    uint32_t            CurrentTimestamp;
    uint32_t            WTime_sec;
} SysTimers;



#if(USER_TIMERS_QUANT > 0)
static void UserTimers(void);
static void UserTimersEventHandler(uint8_t timer);
#endif


/* Sisteminiai taimeriai. Vykdom is SysTick hendlerio (periodas 1 ms)*/
void SysTimeCounterUpdate(void) {

    static uint32_t time = 0u;

    SysTimers.CurrentTimestamp = BSP_GetTick();

    if( time <= SysTimers.CurrentTimestamp ) {
        time = SysTimers.CurrentTimestamp + 1000u;
        SysTimers.WTime_sec++;
    }

#if(USER_TIMERS_QUANT > 0)
    UserTimers();
#endif

}

/* grazina WTime */
uint32_t GetWTime(void) {
    return SysTimers.WTime_sec;
}

/* pradinis nustatymas WTime (pvz. is EEPROM) */
void SetWTime(uint32_t wtime) {
    SysTimers.WTime_sec = wtime;
}

/*  */
uint32_t GetTimestamp(void) {
    return SysTimers.CurrentTimestamp;
}



/* USER taimeriai */
#if(USER_TIMERS_QUANT > 0)

/* inicializuojam taimerius */
void UserTimersInit(void) {

    uint8_t t = 0;

    do {
        SysTimers.user_timers[t].counter = 0;
        SysTimers.user_timers[t].period = 0;
        SysTimers.user_timers[t].is_count = DISABLE;
        SysTimers.user_timers[t].flag = RESET;
    } while(++t < USER_TIMERS_QUANT);

    //uint8_t qqq = sizeof(SysTimers);
}


/* UserTimers kontroleris */
static void UserTimers(void) {

    uint8_t t = 0;

    do {
        if(SysTimers.user_timers[t].is_count != DISABLE) {

            SysTimers.user_timers[t].counter--;

            if(SysTimers.user_timers[t].counter == 0) {
                if(SysTimers.user_timers[t].period != 0) {
                    SysTimers.user_timers[t].counter = SysTimers.user_timers[t].period;
                } else {
                    SysTimers.user_timers[t].is_count = DISABLE;
                }

                SysTimers.user_timers[t].flag = SET;

                UserTimersEventHandler(t);
            }
        }
    } while(++t < USER_TIMERS_QUANT);
}

/* startuojam #taimeri */
void StartUserPeriodicTimer(uint8_t timer, uint16_t period) {
    SysTimers.user_timers[timer].is_count = ENABLE;
    SysTimers.user_timers[timer].period = period;
    SysTimers.user_timers[timer].counter = SysTimers.user_timers[timer].period;
}

/*  */
void StartUserOneShotTimer(uint8_t timer, uint16_t counter) {
    SysTimers.user_timers[timer].is_count = ENABLE;
    SysTimers.user_timers[timer].counter = counter;
}



/* stabdom #taimeri */
void StopUserTimer(uint8_t timer) {
    SysTimers.user_timers[timer].counter = 0;
    SysTimers.user_timers[timer].is_count = DISABLE;
}

/* laikinai pristabdom #taimeri */
void PauseUserTimer(uint8_t timer) {
    SysTimers.user_timers[timer].is_count = DISABLE;
}

/* paleidziam po laikino sustabdymo #taimeri */
void ResumeUserTimer(uint8_t timer) {
    if(SysTimers.user_timers[timer].counter == 0) return;
    SysTimers.user_timers[timer].is_count = ENABLE;
}

/* grazinam vartotojo taimerio perioda */
uint16_t GetUserTimerPeriod(uint8_t timer) {
    return SysTimers.user_timers[timer].period;
}

/* grazinam vartotojo taimerio flaga */
FlagStatus GetUserTimerFlag(uint8_t timer) {

    FlagStatus flag = SysTimers.user_timers[timer].flag;
    SysTimers.user_timers[timer].flag = RESET;
    return flag;
}

/* vartotoju taimeriu hendleris.  */
static void UserTimersEventHandler(uint8_t timer) {

    switch(timer) {
    case USER_TIMER1:

        break;
    case USER_TIMER2:

        break;
    case USER_TIMER3:

        break;
    case USER_TIMER4:

        StartUserOneShotTimer(USER_TIMER2, 50);
        break;
    }
}

#endif
