#ifndef SOFTWARE_H_INCLUDED
#define SOFTWARE_H_INCLUDED

#include "common.h"
#include "board.h"





void                SystemDataInit(void);
void                MbDataInit(void);
void                ReadDipSwitch(void);
void                EEDataRestore(void);
void                SystemDataUpdate(void);
void                MbDataUpdate(void);
void                ReadAnalogInputs(void);
void                MotorConfig( const MotorParamSet* preset );
void                CoolerController(void);
void                CoolerOnByTime(uint8_t sec);
void                RelayController(void);
void                LedsController(void);
void                SystemReset(void);


uint8_t             CheckBaudrateValue(uint32_t baudrate);
uint8_t             CheckBaudrateIndex( uint8_t idx );

uint8_t             GetIndexByBaudrate( uint32_t baudrate );    // grazina bodreito indeksa
uint32_t            GetBaudrateByIndex( uint8_t idx );  // grazina bodreita pagal jo indeksa

uint8_t             GetCurrentBaudrateIndex( void );    // grazina aktyvaus bodreito indeksa
uint8_t             GetCurrentParity( void );           // grazina aktyvu parity reiksme
uint8_t             GetCurrentStopBits( void );
uint8_t             GetCurrentDataBits( void );

uint8_t             InverseBits(uint8_t data);

#endif /* SOFTWARE_H_INCLUDED */
