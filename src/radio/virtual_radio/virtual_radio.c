/*!
 * \file      virtual_radio.c
 *
 * \brief     virtual_radio driver implementation
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
#include <math.h>
#include <string.h>
#include "utilities.h"
#include "timer.h"
#include "radio.h"
#include "delay.h"
#include "virtual_radio.h"
#include "virtual_radio-board.h"
#include "stdio.h"
#include "log.h"

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
Virtual_Radio_t Virtual_Radio;


/*
 * Radio driver functions implementation
 */

void Virtual_Radio_Init( RadioEvents_t *events )
{
    log_debug("");
}

RadioState_t Virtual_Radio_GetStatus( void )
{
    log_debug("");
    return Virtual_Radio.Settings.State;
}

void Virtual_Radio_SetChannel( uint32_t freq )
{
   log_debug("");
}

bool Virtual_Radio_IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool status = true;
    log_debug("");
    return status;
}

uint32_t Virtual_Radio_Random( void )
{
    uint32_t rnd = 0;
    log_debug("");
    return rnd;
}



void Virtual_Radio_SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    log_debug("");
}

void Virtual_Radio_SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    log_debug("");
}

uint32_t Virtual_Radio_GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;
    log_debug("");
    return airTime;
}

void Virtual_Radio_Send( uint8_t *buffer, uint8_t size )
{
    log_debug("");
}

void Virtual_Radio_SetSleep( void )
{
   log_debug("");
}

void Virtual_Radio_SetStby( void )
{
   log_debug("");
}

void Virtual_Radio_SetRx( uint32_t timeout )
{
    log_debug("");
}

void Virtual_Radio_SetTx( uint32_t timeout )
{
    log_debug("");
}

void Virtual_Radio_StartCad( void )
{
    log_debug("");
}

void Virtual_Radio_SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
   log_debug("");
}

int16_t Virtual_Radio_ReadRssi( RadioModems_t modem )
{
    int16_t rssi = 0;
    log_debug("");
    return rssi;
}

void Virtual_Radio_SetModem( RadioModems_t modem )
{
    log_debug("");
}

void Virtual_Radio_Write( uint16_t addr, uint8_t data )
{
    log_debug("");
}

uint8_t Virtual_Radio_Read( uint16_t addr )
{
    log_debug("");
}

void Virtual_Radio_WriteBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    log_debug("");
}

void Virtual_Radio_ReadBuffer( uint16_t addr, uint8_t *buffer, uint8_t size )
{
    log_debug("");
}


void Virtual_Radio_SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    log_debug("");
}

void Virtual_Radio_SetPublicNetwork( bool enable )
{
    log_debug("");
}

uint32_t Virtual_Radio_GetWakeupTime( void )
{
    log_debug("");
}

