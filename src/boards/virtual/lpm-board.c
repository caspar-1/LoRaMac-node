/*!
 * \file      lpm-board.c
 *
 * \brief     Target board low power modes management
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
 *              (C)2013-2017 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team (C)( STMicroelectronics International )
 */
#include <stdint.h>
#include "utilities.h"
#include "lpm-board.h"

static uint32_t StopModeDisable = 0;
static uint32_t OffModeDisable = 0;

void LpmSetOffMode( LpmId_t id, LpmSetMode_t mode )
{

}

void LpmSetStopMode( LpmId_t id, LpmSetMode_t mode )
{

}

void LpmEnterLowPower( void )
{
  
}

LpmGetMode_t LpmGetMode(void)
{
    LpmGetMode_t mode;
    return mode;
}
/*
__weak void LpmEnterSleepMode( void )
{
}

__weak void LpmExitSleepMode( void )
{
}

__weak void LpmEnterStopMode( void )
{
}

__weak void LpmExitStopMode( void )
{
}

__weak void LpmEnterOffMode( void )
{
}

__weak void LpmExitOffMode( void )
{
}
*/