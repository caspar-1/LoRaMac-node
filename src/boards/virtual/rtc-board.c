/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
#include <math.h>
#include <time.h>
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include <unistd.h>
#include "log.h"
#include <time.h>
#include <stdlib.h>
#include <pthread.h>

// MCU Wake Up Time
#define MIN_ALARM_DELAY 10 // in ms

// sub-second number of bits
#define N_PREDIV_S 10

// Synchronous prediv
#define PREDIV_S ((1 << N_PREDIV_S) - 1)

// Asynchronous prediv
#define PREDIV_A (1 << (15 - N_PREDIV_S)) - 1

// Sub-second mask definition
#define ALARM_SUBSECOND_MASK (N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos)

// RTC Time base in us
#define USEC_NUMBER 1000000
#define MSEC_NUMBER (USEC_NUMBER / 1000)

#define COMMON_FACTOR 3
#define CONV_NUMER (MSEC_NUMBER >> COMMON_FACTOR)
#define CONV_DENOM (1 << (N_PREDIV_S - COMMON_FACTOR))

/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC(X, N) (((X) + (N)-1) / (N))

#define ALARM_TICk_TIME_mS 10
pthread_t thread_timer_loop;

static uint32_t alarm_timer = 0;
static uint32_t alarm_timer_set = 0;
static uint32_t now = 0;

void *RTC_thread_function(void *ptr)
{
    while (1)
    {
        usleep(ALARM_TICk_TIME_mS * 1000);
        now++;
        if (alarm_timer == now)
        {
            log_debug("F:%s Timer Fired!", __func__);
            TimerIrqHandler();
        }
    }
}

void RtcInit(void)
{
    log_debug("F:%s", __func__);
    pthread_create(&thread_timer_loop, NULL, RTC_thread_function, (void *)0);
}

uint32_t RtcSetTimerContext(void)
{

    alarm_timer_set = now;
    log_debug("F:%s --> %d", __func__, alarm_timer_set);
    return alarm_timer_set;
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In ticks
 */
uint32_t RtcGetTimerContext(void)
{
    log_debug("F:%s -->%d", __func__, alarm_timer_set);
    return alarm_timer_set;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return ( uint32_t )( ( ( ( uint64_t )milliseconds ) * CONV_DENOM ) / CONV_NUMER );
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
    uint32_t seconds = tick >> N_PREDIV_S;

    tick = tick & PREDIV_S;
    return ( ( seconds * 1000 ) + ( ( tick * 1000 ) >> N_PREDIV_S ) );
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{

}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    
}

void RtcStopAlarm( void )
{
  
}

void RtcStartAlarm( uint32_t timeout )
{
}

uint32_t RtcGetTimerValue( void )
{
    return( 0 );
}

uint32_t RtcGetTimerElapsedTime( void )
{
}

void RtcSetMcuWakeUpTime( void )
{
}

int16_t RtcGetMcuWakeUpTime( void )
{
}


uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    return 0;
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
}

void RtcProcess( void )
{
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
}
