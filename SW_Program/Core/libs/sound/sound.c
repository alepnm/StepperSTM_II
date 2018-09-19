/**
  ******************************************************************************
  * File Name          : sound.c
  * Description        :
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "board.h"
#include "sound.h"

/* Defines, macro, typedefs --------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef* pBeeperTimer;

/* Global variables ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SND_HandleTypeDef Sounder = {
    .tone = 1,
    .freq = 1000,
    .volume = LOW,
    .enable = false
};

/* Private functions ---------------------------------------------------------*/


/**
*/
void SoundInit( bool ena ) {

    Sounder.enable = ena;

    Sounder.volume = LOW;
    Sounder.freq = 1000;
//    Sounder.tone = 1;

    BSP_SoundTimerInit();
}


/** garso stiprumo nustatymas
 */
void SoundSetVolume( SND_VolTypeDef volume ) {
    Sounder.volume = volume;
    BSP_SoundTimerInit();
}


/** garso daznio nustatymas
 */
void SoundSetFreq( uint16_t freq ) {
    Sounder.freq = freq;
    BSP_SoundTimerInit();
}


/** startuojam garsa
 */
void SoundStart( uint8_t tone ) {
    Sounder.tone = tone;
}


/* vykdom is SysTick interaptu kas 1ms */
void SoundHandler() {
    static uint8_t stage, cnt = (1u), begin = true;

    if(!Sounder.enable || !Sounder.tone) {
        return;
    }

    if(begin) {
        begin = false;
        stage = (0u);
    } else {
        if(--cnt) return;
    }
    if(Sounder.tone == (1u)) {
        //SoundSetFreq(F1KHZ);
        BSP_SoundTimerStart();
        switch(stage) {
        case 0:
            cnt = (30u);
            break;
        case 1:
        default:
            goto stop;
        }
        stage++;
        return;
    }
    if(Sounder.tone == (2u)) {
        //SoundSetFreq(1000);
        BSP_SoundTimerStart();
        switch(stage) {
        case 0:
            cnt = (50u);
            BSP_SoundTimerStart();
            break;
        case 1:
            cnt = (50u);
            BSP_SoundTimerStop();
            break;
        case 2:
            cnt = (50u);
            BSP_SoundTimerStart();
            break;
        case 3:
            cnt = (50u);
            BSP_SoundTimerStop();
            break;
        case 4:
        default:
            goto stop;
        }
        stage++;
        return;
    }
    if(Sounder.tone == (3u)) {
        //SoundSetFreq(F1KHZ);
        switch(stage) {
        case 0:
            cnt = (50u);
            BSP_SoundTimerStart();
            break;
        case 1:
            cnt = (50u);
            BSP_SoundTimerStop();
            break;
        case 2:
            cnt = (50u);
            BSP_SoundTimerStart();
            break;
        case 3:
            cnt = (50u);
            BSP_SoundTimerStop();
            break;
        case 4:
            cnt = (50u);
            BSP_SoundTimerStart();
            break;
        case 5:
        default:
            //Sounder.freq = F1KHZ;
            goto stop;
        }
        if(stage < 5) {
            stage++;
        } else {
            stage = 5;
        }
        return;
    }
    if(Sounder.tone == (4u)) {
        BSP_SoundTimerStart();
        switch(stage) {
        case 0:
            cnt = (40u);
            SoundSetFreq(1000);
            break;
        case 1:
            cnt = (40u);
            SoundSetFreq(1000-25);
            break;
        case 2:
            cnt = (40u);
            SoundSetFreq(1000-50);
            break;
        case 3:
            cnt = (40u);
            SoundSetFreq(1000-75);
            break;
        case 4:
            cnt = (40u);
            SoundSetFreq(1000-100);
            break;
        case 5:
            cnt = (40u);
            SoundSetFreq(1000-125);
            break;
        case 6:
            cnt = (40u);
            SoundSetFreq(1000-150);
            break;
        case 7:
            cnt = (40u);
            SoundSetFreq(1000-175);
            break;
        case 8:
        default:
            SoundSetFreq(1000);
            goto stop;
        }
        stage++;
        return;
    }
stop:
    BSP_SoundTimerStop();
    stage = (0u);
    Sounder.tone = (0u);
    begin = true;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
