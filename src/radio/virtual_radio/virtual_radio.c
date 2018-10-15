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

void Virtual_Radio_Init(RadioEvents_t *events)
{
    log_debug("F:%s", __func__);
}

RadioState_t Virtual_Radio_GetStatus(void)
{
    log_debug("F:%s", __func__);
    return Virtual_Radio.Settings.State;
}

void Virtual_Radio_SetChannel(uint32_t freq)
{
    log_debug("F:%s (freq=%d)", __func__, freq);
}

bool Virtual_Radio_IsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime)
{
    bool status = true;
    log_debug("F:%s (freq=%d,rssiThresh=%d,maxCarrierSenseTime=%d)", __func__, freq, rssiThresh, maxCarrierSenseTime);
    return status;
}

uint32_t Virtual_Radio_Random(void)
{
    uint32_t rnd = 0;
    log_debug("F:%s", __func__);
    return rnd;
}

void Virtual_Radio_SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                               uint32_t datarate, uint8_t coderate,
                               uint32_t bandwidthAfc, uint16_t preambleLen,
                               uint16_t symbTimeout, bool fixLen,
                               uint8_t payloadLen,
                               bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                               bool iqInverted, bool rxContinuous)
{
    log_debug("F:%s (\
    bandwidth=%d,\
    datarate=%d,\
    coderate=%d,\
    bandwidthAfc=%d,\
    preambleLen=%d,\
    symbTimeout=%d,\
    fixLen=%d,\
    payloadLen=%d,\
    crcOn=%d,\
    freqHopOn=%d,\
    hopPeriod=%d,\
    iqInverted=%d,\
    rxContinuous=%d)",
              __func__, bandwidth, datarate, coderate, bandwidthAfc, preambleLen, symbTimeout, fixLen, payloadLen, crcOn, freqHopOn, hopPeriod, iqInverted, rxContinuous);
}

void Virtual_Radio_SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                               uint32_t bandwidth, uint32_t datarate,
                               uint8_t coderate, uint16_t preambleLen,
                               bool fixLen, bool crcOn, bool freqHopOn,
                               uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    log_debug("F:%s (\
    power=%d,\
    fdev=%d,\
    bandwidth=%d,\
    datarate=%d,\
    coderate=%d,\
    preambleLen=%d,\
    fixLen=%d,\
    crcOn=%d,\
    freqHopOn=%d,\
    hopPeriod=%d,\
    iqInverted=%d,\
    timeout=%d\
    )", __func__,power, fdev, bandwidth, datarate, coderate, preambleLen, fixLen, crcOn, freqHopOn, hopPeriod, iqInverted, timeout);
}

uint32_t Virtual_Radio_GetTimeOnAir(RadioModems_t modem, uint8_t pktLen)
{
    uint32_t airTime = 0;
    log_debug("F:%s", __func__);
    return airTime;
}

void Virtual_Radio_Send(uint8_t *buffer, uint8_t size)
{
    log_debug("F:%s <%s>", __func__, log_helper_buffer_to_str(buffer, size));
}

void Virtual_Radio_SetSleep(void)
{
    log_debug("F:%s", __func__);
}

void Virtual_Radio_SetStby(void)
{
    log_debug("F:%s", __func__);
}

void Virtual_Radio_SetRx(uint32_t timeout)
{
    log_debug("F:%s (timeout=%d)", __func__, timeout);
}

void Virtual_Radio_SetTx(uint32_t timeout)
{
    log_debug("F:%s (timeout=%d)", __func__, timeout);
}

void Virtual_Radio_StartCad(void)
{
    log_debug("F:%s", __func__);
}

void Virtual_Radio_SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    log_debug("F:%s  (freq=%d,power=%d,time=%d)", __func__, freq, power, time);
}

int16_t Virtual_Radio_ReadRssi(RadioModems_t modem)
{
    int16_t rssi = 0;
    log_debug("F:%s", __func__);
    return rssi;
}

void Virtual_Radio_SetModem(RadioModems_t modem)
{
    log_debug("F:%s", __func__);
}

void Virtual_Radio_Write(uint16_t addr, uint8_t data)
{
    log_debug("F:%s  (addr=%d,data=%d)", __func__, addr, data);
}

uint8_t Virtual_Radio_Read(uint16_t addr)
{
    log_debug("F:%s  (addr=%d)", __func__, addr);
}

void Virtual_Radio_WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    log_debug("F:%s (addr=%d,data=<%s>)", __func__, log_helper_buffer_to_str(buffer, size));
}

void Virtual_Radio_ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    log_debug("F:%s", __func__);
}

void Virtual_Radio_SetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    log_debug("F:%s (max=%d)", __func__, max);
}

void Virtual_Radio_SetPublicNetwork(bool enable)
{
    log_debug("F:%s (enable=%d)", __func__, enable);
}

uint32_t Virtual_Radio_GetWakeupTime(void)
{
    log_debug("F:%s", __func__);
}
