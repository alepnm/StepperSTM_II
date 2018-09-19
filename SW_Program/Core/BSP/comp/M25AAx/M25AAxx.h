/*

*/

#ifndef MICROCHIP_25AA02_H
#define MICROCHIP_25AA02_H

#include <stdbool.h>
#include <stdint.h>


#define EE_INIT_BYTE                    0x55

#define EE_SIZE                         192

#define EEADDR_BASE                     0

#define EEADDR_INIT_BYTE                EEADDR_BASE+3

#define EEADR_MBADDR                    EEADDR_BASE+5                   // byte
#define EEADDR_MBBAUDRATE               EEADR_MBADDR+1                  // byte
#define EEADR_PARITY                    EEADDR_MBBAUDRATE+1             // byte
#define EEADR_STOPBITS                  EEADR_PARITY+1                  // byte
#define EEADR_SCROLL_RPM                EEADR_STOPBITS+1                // byte
#define EEADR_HS_TO_VALUE               EEADR_SCROLL_RPM+1              // byte
#define EEADR_SCROLL_OFF_CYCLE_TIME     EEADR_HS_TO_VALUE+1             // word
#define EEADR_SCROLL_ON_CYCLE_TIME      EEADR_SCROLL_OFF_CYCLE_TIME+2   // word
#define EEADR_SCROLL_SYNC               EEADR_SCROLL_ON_CYCLE_TIME+2    // byte
#define EEADR_MICROSTEPS                EEADR_SCROLL_SYNC+1             // byte
#define EEADR_USERSET_STEPS_PER_REV     EEADR_MICROSTEPS+1              // byte
#define EEADR_USERSET_KVAL_RUN          EEADR_USERSET_STEPS_PER_REV+1   // byte
#define EEADR_USERSET_KVAL_ACC          EEADR_USERSET_KVAL_RUN+1        // byte
#define EEADR_USERSET_KVAL_DEC          EEADR_USERSET_KVAL_ACC+1        // byte
#define EEADR_USERSET_KVAL_HOLD         EEADR_USERSET_KVAL_DEC+1        // byte
#define EEADR_USERSET_TRES_OCD          EEADR_USERSET_KVAL_HOLD+1       // word
#define EEADR_USERSET_TRES_STALL        EEADR_USERSET_TRES_OCD+2        // word
#define EEADR_MIN_RPM                   EEADR_USERSET_TRES_STALL+2      // byte
#define EEADR_MAX_RPM                   EEADR_MIN_RPM+1                 // byte
#define EEADR_USERSET_SPEED_ACC         EEADR_MAX_RPM+1                 // word
#define EEADR_USERSET_SPEED_DEC         EEADR_USERSET_SPEED_ACC+2       // word
#define EEADR_OVH_TIMEOUT               EEADR_USERSET_SPEED_DEC+2       // word
#define EEADR_SOUND_LEVEL               EEADR_OVH_TIMEOUT+2             // byte
#define EEADR_WTIME                     EEADR_SOUND_LEVEL+1             // dword
#define EEADR_TRANSMISSION_RATIO        EEADR_WTIME+4                   // byte
#define EEADR_WDT_FUNC                  EEADR_TRANSMISSION_RATIO+1      // byte

bool        M25AAxx_ReadUID( unsigned char* buffer);
uint8_t     M25AAxx_ReadByte(uint8_t addr);
uint16_t    M25AAxx_ReadWord(uint8_t addr);
uint32_t    M25AAxx_ReadDWord(uint8_t addr);
bool        M25AAxx_Read( uint8_t addr, uint8_t* data, uint8_t len );
bool        M25AAxx_WriteByte( uint8_t addr, uint8_t value );
bool        M25AAxx_WriteWord( uint8_t addr, uint16_t value );
bool        M25AAxx_WriteDWord( uint8_t addr, uint32_t value );
bool        M25AAxx_Write(uint8_t addr, uint8_t* data, uint8_t len);
uint8_t     M25AAxx_Clear(void);



#endif // MICROCHIP_25AA02_H
