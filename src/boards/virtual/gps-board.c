/*!
 * \file      gps-board.c
 *
 * \brief     Target board GPS driver implementation
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
#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "uart.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "gps-board.h"

/*!
 * FIFO buffers size
 */
//#define FIFO_TX_SIZE                                128
#define FIFO_RX_SIZE                                128

//uint8_t TxBuffer[FIFO_TX_SIZE];
uint8_t RxBuffer[FIFO_RX_SIZE];

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
uint8_t NmeaStringSize = 0;

Gpio_t GpsPowerEn;

bool GpsPowerEnInverted = false;

extern Uart_t Uart1;

void GpsMcuOnPpsSignal( void )
{
    
}

void GpsMcuInvertPpsTrigger( void )
{

}

void GpsMcuInit( void )
{
    
}

void GpsMcuStart( void )
{
    
}

void GpsMcuStop( void )
{
  
}

void GpsMcuProcess( void )
{

}

void GpsMcuIrqNotify( UartNotifyId_t id )
{
    
}
