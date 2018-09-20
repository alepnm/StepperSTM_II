#ifndef SOUND_H_INCLUDED
#define SOUND_H_INCLUDED

/**
  ******************************************************************************
  * File Name          : sound.h
  * Description        :
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
//#include <stdbool.h>
//#include "stm32f0xx_hal.h"
/* Defines, macro, typedefs --------------------------------------------------*/


typedef enum { SND_OFF = 0, SND_LOW = 1, SND_MID = 5, SND_HIGH = 20 } SND_VolTypeDef;  //procentais


typedef struct {
    uint8_t		        tone    :3;    	// cypso tonas (0-7). Jai 0 - nera cypso
    uint16_t            freq;    // cypso daznis
    SND_VolTypeDef      volume;  // cypso stiprumas 3-ju lygiu
    bool                enable;
} SND_HandleTypeDef;

extern SND_HandleTypeDef Sounder;


/* Global functions ----------------------------------------------------------*/
void SoundHandler( void );
void SoundInit( bool ena );
void SoundStart( uint8_t tone );
void SoundSetVolume( SND_VolTypeDef volume );
void SoundSetFreq( uint16_t freq );



#endif /* SOUND_H_INCLUDED */

/************************ (C) COPYRIGHT STMicroelectronics ********************/

