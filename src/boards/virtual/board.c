/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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

#include "utilities.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "gps.h"
#include "mpl3115.h"
#include "mag3110.h"
#include "mma8451.h"
#include "sx9500.h"
#include "board-config.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "sx1272-board.h"
#include "board.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <pthread.h>

/*!
 * LED GPIO pins objects
 */

Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;
Gpio_t Led4;

/*
 * MCU objects
 */
Adc_t Adc;
I2c_t I2c;
Uart_t Uart1;

static pthread_mutex_t cs_mutex =  PTHREAD_MUTEX_INITIALIZER;



/*!
 * Holds the bord version.
 */
static Version_t BoardVersion = {0};

void BoardCriticalSectionBegin(uint32_t *mask)
{
    pthread_mutex_lock( &cs_mutex );
}

void BoardCriticalSectionEnd(uint32_t *mask)
{
    pthread_mutex_unlock( &cs_mutex );
}

void BoardInitPeriph(void)
{
  
}

void BoardInitMcu(void)
{
    RtcInit();
}

void BoardResetMcu(void)
{
}

void BoardDeInitMcu(void)
{
}

uint32_t BoardGetRandomSeed(void)
{
    return (uint32_t)rand();
}

void BoardGetUniqueId(uint8_t *id)
{
    id[7] = 6;
    id[6] = 5;
    id[5] = 4;
    id[4] = 0xff;
    id[3] = 0xff;
    id[2] = 3;
    id[1] = 2;
    id[0] = 1;
}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY 3300 // mV

/*!
 * VREF calibration value
 */
#define VREFINT_CAL (*(uint16_t *)0x1FF80078)

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE 4095

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL 3700      // mV
#define BATTERY_MIN_LEVEL 1900      // mV
#define BATTERY_SHUTDOWN_LEVEL 1800 // mV

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

uint16_t BoardBatteryMeasureVolage(void)
{
    return 0;
}

uint32_t BoardGetBatteryVoltage(void)
{
    return 0;
}

uint8_t BoardGetBatteryLevel(void)
{
    uint8_t batteryLevel = 0;
    return batteryLevel;
}

static void BoardUnusedIoInit(void)
{
}

Version_t BoardGetVersion(void)
{
    Version_t boardVersion = {0};
    boardVersion.Value = 0;
    return boardVersion;
}

void SystemClockConfig(void)
{
}

void CalibrateSystemWakeupTime(void)
{
}

void SystemClockReConfig(void)
{
}

void SysTick_Handler(void)
{
}

uint8_t GetBoardPowerSource(void)
{
    return BATTERY_POWER;
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode(void)
{
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode(void)
{
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode(void)
{
}

void BoardLowPowerHandler(void)
{
}
