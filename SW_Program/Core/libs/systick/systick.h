#ifndef SYSTICK_H_INCLUDED
#define SYSTICK_H_INCLUDED


#define USER_TIMER1     0
#define USER_TIMER2     1
#define USER_TIMER3     2
#define USER_TIMER4     3


/* API funkcijos */
void        SysTimeCounterUpdate(void);
uint32_t    GetWTime(void);
void        SetWTime(uint32_t wtime);
uint32_t    GetTimestamp(void);

void        UserTimersInit(void);
void        StartUserPeriodicTimer(uint8_t timer, uint16_t period);
void        StartUserOneShotTimer(uint8_t timer, uint16_t counter);
void        StopUserTimer(uint8_t timer);
void        PauseUserTimer(uint8_t timer);
void        ResumeUserTimer(uint8_t timer);
uint16_t    GetUserTimerPeriod(uint8_t timer);
FlagStatus  GetUserTimerFlag(uint8_t timer);


#endif /* SYSTICK_H_INCLUDED */
