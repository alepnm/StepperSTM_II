/*
  * FreeModbus Libary: LPC214X Port
  * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
//#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    if( xRxEnable ) {
        //SLAVE_RS485_RECEIVE_MODE;
        BSP_UartRxEnable(MbPortParams.Uart);
    } else {
        BSP_UartRxDisable(MbPortParams.Uart);
    }

    if( xTxEnable ) {
        //SLAVE_RS485_SEND_MODE;
        BSP_UartTxEnable(MbPortParams.Uart);
    } else {
        BSP_UartTxDisable(MbPortParams.Uart);
    }
}


BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    if(ucPORT >= 2) return FALSE;

    MbPortParams.Uart = ucPORT;

    if( (uint8_t)BSP_UartConfig( (uint8_t)ucPORT, (uint32_t)ulBaudRate, (uint8_t)ucDataBits, (uint8_t)eParity ) != 0 ) return FALSE;
    if( (uint8_t)BSP_UartStart( (uint8_t)ucPORT ) != 0 ) return FALSE;

    vMBPortSerialEnable( TRUE, FALSE );

    return TRUE;
}

BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    BSP_PutByteToUart(MbPortParams.Uart, ucByte);
    return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte ){

    *pucByte = BSP_GetReceivedByte(MbPortParams.Uart);

    return TRUE;
}

/*
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
//static void prvvUARTTxReadyISR( void )
//{
//    pxMBFrameCBTransmitterEmpty(  );
//}

/*
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
//static void prvvUARTRxISR( void )
//{
//    pxMBFrameCBByteReceived(  );
//}

