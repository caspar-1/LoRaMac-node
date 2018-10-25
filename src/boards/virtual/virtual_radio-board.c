/*!
 * \file      Virtual_Radio_-board.c
 *
 * \brief     Target board Virtual_Radio_ driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "delay.h"
#include "radio.h"
#include "virtual_radio-board.h"
#include "log.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    Virtual_Radio_Init,
    Virtual_Radio_GetStatus,
    Virtual_Radio_SetModem,
    Virtual_Radio_SetChannel,
    Virtual_Radio_IsChannelFree,
    Virtual_Radio_Random,
    Virtual_Radio_SetRxConfig,
    Virtual_Radio_SetTxConfig,
    Virtual_Radio_CheckRfFrequency,
    Virtual_Radio_GetTimeOnAir,
    Virtual_Radio_Send,
    Virtual_Radio_SetSleep,
    Virtual_Radio_SetStby,
    Virtual_Radio_SetRx,
    Virtual_Radio_StartCad,
    Virtual_Radio_SetTxContinuousWave,
    Virtual_Radio_ReadRssi,
    Virtual_Radio_Write,
    Virtual_Radio_Read,
    Virtual_Radio_WriteBuffer,
    Virtual_Radio_ReadBuffer,
    Virtual_Radio_SetMaxPayloadLength,
    Virtual_Radio_SetPublicNetwork,
    Virtual_Radio_GetWakeupTime,
    NULL, // void ( *IrqProcess )( void )
    NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
    NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};


bool Virtual_Radio_CheckRfFrequency( uint32_t frequency )
{
    log_debug("F:%s (frequency=%d)", __func__, frequency);
    return true;
}
