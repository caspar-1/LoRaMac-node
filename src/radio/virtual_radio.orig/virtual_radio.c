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
 * 
 * \author    Caspar Lucas () 
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

#define REG_PAYLOADLENGTH 0x32
#define RSSI_OFFSET -139

/*!
 * \brief Tx & Rx timeout timer callback
 */
void Virtual_Radio_OnTimeout_Async(void);

/*
 * Public global variables
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;
TimerEvent_t RxTimeoutSyncWord;

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*!
 * Radio hardware and global parameters
 */
Virtual_Radio_t Virtual_Radio;

/*
* stub functions
*/
uint16_t get_rssi()
{
    return 0;
}

static uint8_t RADIO_REGS[256];

uint8_t SX1272Read(uint8_t REG)
{
    return RADIO_REGS[REG];
}

void SX1272Write(uint8_t REG, uint8_t VALUE)
{
}

void SX1272SetSleep()
{

}

void SX1272SetRfTxPower( int8_t power )
{

}

void SX1272ReadFifo()
{
    
}



void SX1272SetModem(RadioModems_t modem)
{
    if ((SX1272Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0)
    {
        Virtual_Radio.Settings.Modem = MODEM_LORA;
    }
    else
    {
        Virtual_Radio.Settings.Modem = MODEM_FSK;
    }

    if (Virtual_Radio.Settings.Modem == modem)
    {
        return;
    }

    Virtual_Radio.Settings.Modem = modem;
    switch (Virtual_Radio.Settings.Modem)
    {
    default:
    case MODEM_FSK:
        SX1272SetSleep();
        SX1272Write(REG_OPMODE, (SX1272Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

        SX1272Write(REG_DIOMAPPING1, 0x00);
        SX1272Write(REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1272SetSleep();
        SX1272Write(REG_OPMODE, (SX1272Read(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

        SX1272Write(REG_DIOMAPPING1, 0x00);
        SX1272Write(REG_DIOMAPPING2, 0x00);
        break;
    }
}

/*
 * Radio driver functions implementation
 */

void Virtual_Radio_Init(RadioEvents_t *events)
{
    uint8_t i;

    RadioEvents = events;

    // Initialize driver timeout timers
    TimerInit(&TxTimeoutTimer, Virtual_Radio_OnTimeout_Async);
    TimerInit(&RxTimeoutTimer, Virtual_Radio_OnTimeout_Async);
    TimerInit(&RxTimeoutSyncWord, Virtual_Radio_OnTimeout_Async);

    //SX1272Reset( );

    //SX1272SetOpMode( RF_OPMODE_SLEEP );

    //SX1272IoIrqInit( DioIrq );
    /*
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1272SetModem( RadioRegsInit[i].Modem );
        SX1272Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }
*/
    SX1272SetModem(MODEM_FSK);

    Virtual_Radio.Settings.State = RF_IDLE;

    log_debug("F:%s", __func__);
}

RadioState_t Virtual_Radio_GetStatus(void)
{
    RadioState_t _st = Virtual_Radio.Settings.State;
    log_debug("F:%s --> %d", __func__, _st);
    return _st;
}

void Virtual_Radio_SetChannel(uint32_t freq)
{
    Virtual_Radio.Settings.Channel = freq;
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

    SX1272SetModem(modem);

    switch (modem)
    {
    case MODEM_FSK:
    {
        Virtual_Radio.Settings.Fsk.Bandwidth = bandwidth;
        Virtual_Radio.Settings.Fsk.Datarate = datarate;
        Virtual_Radio.Settings.Fsk.BandwidthAfc = bandwidthAfc;
        Virtual_Radio.Settings.Fsk.FixLen = fixLen;
        Virtual_Radio.Settings.Fsk.PayloadLen = payloadLen;
        Virtual_Radio.Settings.Fsk.CrcOn = crcOn;
        Virtual_Radio.Settings.Fsk.IqInverted = iqInverted;
        Virtual_Radio.Settings.Fsk.RxContinuous = rxContinuous;
        Virtual_Radio.Settings.Fsk.PreambleLen = preambleLen;
        Virtual_Radio.Settings.Fsk.RxSingleTimeout = (uint32_t)(symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000);

        //datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        //SX1272Write(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        //SX1272Write(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        //SX1272Write(REG_RXBW, GetFskBandwidthRegValue(bandwidth));
        //SX1272Write(REG_AFCBW, GetFskBandwidthRegValue(bandwidthAfc));

        //SX1272Write(REG_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        //SX1272Write(REG_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        //if (fixLen == 1)
        //{
        //    SX1272Write(REG_PAYLOADLENGTH, payloadLen);
        //}
        //else
        //{
        //    SX1272Write(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
        //}
        /*
        SX1272Write(REG_PACKETCONFIG1,
                    (SX1272Read(REG_PACKETCONFIG1) &
                     RF_PACKETCONFIG1_CRC_MASK &
                     RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                        ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                        (crcOn << 4));
        SX1272Write(REG_PACKETCONFIG2, (SX1272Read(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        */
    }
    break;
    case MODEM_LORA:
    {
        Virtual_Radio.Settings.LoRa.Bandwidth = bandwidth;
        Virtual_Radio.Settings.LoRa.Datarate = datarate;
        Virtual_Radio.Settings.LoRa.Coderate = coderate;
        Virtual_Radio.Settings.LoRa.PreambleLen = preambleLen;
        Virtual_Radio.Settings.LoRa.FixLen = fixLen;
        Virtual_Radio.Settings.LoRa.PayloadLen = payloadLen;
        Virtual_Radio.Settings.LoRa.CrcOn = crcOn;
        Virtual_Radio.Settings.LoRa.FreqHopOn = freqHopOn;
        Virtual_Radio.Settings.LoRa.HopPeriod = hopPeriod;
        Virtual_Radio.Settings.LoRa.IqInverted = iqInverted;
        Virtual_Radio.Settings.LoRa.RxContinuous = rxContinuous;

        if (datarate > 12)
        {
            datarate = 12;
        }
        else if (datarate < 6)
        {
            datarate = 6;
        }

        if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
        {
            Virtual_Radio.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            Virtual_Radio.Settings.LoRa.LowDatarateOptimize = 0x00;
        }
        /*
        SX1272Write(REG_LR_MODEMCONFIG1,
                    (SX1272Read(REG_LR_MODEMCONFIG1) &
                     RFLR_MODEMCONFIG1_BW_MASK &
                     RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                     RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                     RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                     RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK) |
                        (bandwidth << 6) | (coderate << 3) |
                        (fixLen << 2) | (crcOn << 1) |
                        Virtual_Radio.Settings.LoRa.LowDatarateOptimize);

        SX1272Write(REG_LR_MODEMCONFIG2,
                    (SX1272Read(REG_LR_MODEMCONFIG2) &
                     RFLR_MODEMCONFIG2_SF_MASK &
                     RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                        (datarate << 4) |
                        ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

        SX1272Write(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

        SX1272Write(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        SX1272Write(REG_LR_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1)
        {
            SX1272Write(REG_LR_PAYLOADLENGTH, payloadLen);
        }

        if (Virtual_Radio.Settings.LoRa.FreqHopOn == true)
        {
            SX1272Write(REG_LR_PLLHOP, (SX1272Read(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            SX1272Write(REG_LR_HOPPERIOD, Virtual_Radio.Settings.LoRa.HopPeriod);
        }

        if (datarate == 6)
        {
            SX1272Write(REG_LR_DETECTOPTIMIZE,
                        (SX1272Read(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF6);
            SX1272Write(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF6);
        }
        else
        {
            SX1272Write(REG_LR_DETECTOPTIMIZE,
                        (SX1272Read(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            SX1272Write(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }*/
    }
    break;
    }
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
    )",
              __func__, power, fdev, bandwidth, datarate, coderate, preambleLen, fixLen, crcOn, freqHopOn, hopPeriod, iqInverted, timeout);

    SX1272SetModem(modem);

    SX1272SetRfTxPower(power);

    switch (modem)
    {
    case MODEM_FSK:
    {
        Virtual_Radio.Settings.Fsk.Power = power;
        Virtual_Radio.Settings.Fsk.Fdev = fdev;
        Virtual_Radio.Settings.Fsk.Bandwidth = bandwidth;
        Virtual_Radio.Settings.Fsk.Datarate = datarate;
        Virtual_Radio.Settings.Fsk.PreambleLen = preambleLen;
        Virtual_Radio.Settings.Fsk.FixLen = fixLen;
        Virtual_Radio.Settings.Fsk.CrcOn = crcOn;
        Virtual_Radio.Settings.Fsk.IqInverted = iqInverted;
        Virtual_Radio.Settings.Fsk.TxTimeout = timeout;
        /*
            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            SX1272Write( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            SX1272Write( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1272Write( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1272Write( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1272Write( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1272Write( REG_PREAMBLELSB, preambleLen & 0xFF );

            SX1272Write( REG_PACKETCONFIG1,
                         ( SX1272Read( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            SX1272Write( REG_PACKETCONFIG2, ( SX1272Read( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
            */
    }
    break;
    case MODEM_LORA:
    {
        Virtual_Radio.Settings.LoRa.Power = power;
        Virtual_Radio.Settings.LoRa.Bandwidth = bandwidth;
        Virtual_Radio.Settings.LoRa.Datarate = datarate;
        Virtual_Radio.Settings.LoRa.Coderate = coderate;
        Virtual_Radio.Settings.LoRa.PreambleLen = preambleLen;
        Virtual_Radio.Settings.LoRa.FixLen = fixLen;
        Virtual_Radio.Settings.LoRa.FreqHopOn = freqHopOn;
        Virtual_Radio.Settings.LoRa.HopPeriod = hopPeriod;
        Virtual_Radio.Settings.LoRa.CrcOn = crcOn;
        Virtual_Radio.Settings.LoRa.IqInverted = iqInverted;
        Virtual_Radio.Settings.LoRa.TxTimeout = timeout;

        if (datarate > 12)
        {
            datarate = 12;
        }
        else if (datarate < 6)
        {
            datarate = 6;
        }
        if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
        {
            Virtual_Radio.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            Virtual_Radio.Settings.LoRa.LowDatarateOptimize = 0x00;
        }
        /*
            if( Virtual_Radio.Settings.LoRa.FreqHopOn == true )
            {
                SX1272Write( REG_LR_PLLHOP, ( SX1272Read( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1272Write( REG_LR_HOPPERIOD, Virtual_Radio.Settings.LoRa.HopPeriod );
            }

            SX1272Write( REG_LR_MODEMCONFIG1,
                         ( SX1272Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK &
                           RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK ) |
                           ( bandwidth << 6 ) | ( coderate << 3 ) |
                           ( fixLen << 2 ) | ( crcOn << 1 ) |
                           Virtual_Radio.Settings.LoRa.LowDatarateOptimize );

            SX1272Write( REG_LR_MODEMCONFIG2,
                        ( SX1272Read( REG_LR_MODEMCONFIG2 ) &
                          RFLR_MODEMCONFIG2_SF_MASK ) |
                          ( datarate << 4 ) );


            SX1272Write( REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1272Write( REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1272Write( REG_LR_DETECTOPTIMIZE,
                             ( SX1272Read( REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1272Write( REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }*/
    }
    break;
    }
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



void Virtual_Radio_OnRxTx_Async()
{
    uint8_t irqFlags=0;

    switch (Virtual_Radio.Settings.State)
    {
    case RF_RX_RUNNING:
        TimerStop( &RxTimeoutTimer );
        // RxDone interrupt
        switch (Virtual_Radio.Settings.Modem)
        {
        case MODEM_FSK:
            if (Virtual_Radio.Settings.Fsk.CrcOn == true)
            {
                
                irqFlags = SX1272Read(REG_IRQFLAGS2);
                if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK)
                {
                    // Clear Irqs
                    SX1272Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                   RF_IRQFLAGS1_PREAMBLEDETECT |
                                                   RF_IRQFLAGS1_SYNCADDRESSMATCH);
                    SX1272Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                    TimerStop(&RxTimeoutTimer);

                    if (Virtual_Radio.Settings.Fsk.RxContinuous == false)
                    {
                        TimerStop(&RxTimeoutSyncWord);
                        Virtual_Radio.Settings.State = RF_IDLE;
                    }
                    else
                    {
                        // Continuous mode restart Rx chain
                        SX1272Write(REG_RXCONFIG, SX1272Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                        TimerStart(&RxTimeoutSyncWord);
                    }

                    if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                    {
                        RadioEvents->RxError();
                    }
                    Virtual_Radio.Settings.FskPacketHandler.PreambleDetected = false;
                    Virtual_Radio.Settings.FskPacketHandler.SyncWordDetected = false;
                    Virtual_Radio.Settings.FskPacketHandler.NbBytes = 0;
                    Virtual_Radio.Settings.FskPacketHandler.Size = 0;
                    break;
                }
            }

            // Read received packet size
            if ((Virtual_Radio.Settings.FskPacketHandler.Size == 0) && (Virtual_Radio.Settings.FskPacketHandler.NbBytes == 0))
            {
                if (Virtual_Radio.Settings.Fsk.FixLen == false)
                {
                    SX1272ReadFifo((uint8_t *)&Virtual_Radio.Settings.FskPacketHandler.Size, 1);
                }
                else
                {
                    Virtual_Radio.Settings.FskPacketHandler.Size = SX1272Read(REG_PAYLOADLENGTH);
                }
                SX1272ReadFifo(RxTxBuffer + Virtual_Radio.Settings.FskPacketHandler.NbBytes, Virtual_Radio.Settings.FskPacketHandler.Size - Virtual_Radio.Settings.FskPacketHandler.NbBytes);
                Virtual_Radio.Settings.FskPacketHandler.NbBytes += (Virtual_Radio.Settings.FskPacketHandler.Size - Virtual_Radio.Settings.FskPacketHandler.NbBytes);
            }
            else
            {
                SX1272ReadFifo(RxTxBuffer + Virtual_Radio.Settings.FskPacketHandler.NbBytes, Virtual_Radio.Settings.FskPacketHandler.Size - Virtual_Radio.Settings.FskPacketHandler.NbBytes);
                Virtual_Radio.Settings.FskPacketHandler.NbBytes += (Virtual_Radio.Settings.FskPacketHandler.Size - Virtual_Radio.Settings.FskPacketHandler.NbBytes);
            }

            TimerStop(&RxTimeoutTimer);

            if (Virtual_Radio.Settings.Fsk.RxContinuous == false)
            {
                Virtual_Radio.Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWord);
            }
            else
            {
                // Continuous mode restart Rx chain
                SX1272Write( REG_RXCONFIG, SX1272Read( REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                TimerStart(&RxTimeoutSyncWord);
            }

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(RxTxBuffer, Virtual_Radio.Settings.FskPacketHandler.Size, Virtual_Radio.Settings.FskPacketHandler.RssiValue, 0);
            }
            Virtual_Radio.Settings.FskPacketHandler.PreambleDetected = false;
            Virtual_Radio.Settings.FskPacketHandler.SyncWordDetected = false;
            Virtual_Radio.Settings.FskPacketHandler.NbBytes = 0;
            Virtual_Radio.Settings.FskPacketHandler.Size = 0;
            break;
        case MODEM_LORA:
        {
            int8_t snr = 0;

            // Clear Irq
            //SX1272Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

            //irqFlags = SX1272Read(REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK)==RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK)
            {
                // Clear Irq
                //SX1272Write(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                if (Virtual_Radio.Settings.LoRa.RxContinuous == false)
                {
                    Virtual_Radio.Settings.State = RF_IDLE;
                }
                TimerStop(&RxTimeoutTimer);

                if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                {
                    RadioEvents->RxError();
                }
                break;
            }

            Virtual_Radio.Settings.LoRaPacketHandler.SnrValue = get_rssi();
            if (Virtual_Radio.Settings.LoRaPacketHandler.SnrValue & 0x80) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                snr = ((~Virtual_Radio.Settings.LoRaPacketHandler.SnrValue + 1) & 0xFF) >> 2;
                snr = -snr;
            }
            else
            {
                // Divide by 4
                snr = (Virtual_Radio.Settings.LoRaPacketHandler.SnrValue & 0xFF) >> 2;
            }

            int16_t rssi = get_rssi();
            if (snr < 0)
            {
                Virtual_Radio.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + (rssi >> 4) +
                                                                     snr;
            }
            else
            {
                Virtual_Radio.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET + rssi + (rssi >> 4);
            }

            //Virtual_Radio.Settings.LoRaPacketHandler.Size = SX1272Read(REG_LR_RXNBBYTES);
            //SX1272Write(REG_LR_FIFOADDRPTR, SX1272Read(REG_LR_FIFORXCURRENTADDR));
            //SX1272ReadFifo(RxTxBuffer, Virtual_Radio.Settings.LoRaPacketHandler.Size);

            if (Virtual_Radio.Settings.LoRa.RxContinuous == false)
            {
                Virtual_Radio.Settings.State = RF_IDLE;
            }
            TimerStop(&RxTimeoutTimer);

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(RxTxBuffer, Virtual_Radio.Settings.LoRaPacketHandler.Size, Virtual_Radio.Settings.LoRaPacketHandler.RssiValue, Virtual_Radio.Settings.LoRaPacketHandler.SnrValue);
            }
        }
        break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        TimerStop(&TxTimeoutTimer);
        // TxDone interrupt
        switch (Virtual_Radio.Settings.Modem)
        {
        case MODEM_LORA:
            // Clear Irq
            //SX1272Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
            // Intentional fall through
        case MODEM_FSK:
        default:
            Virtual_Radio.Settings.State = RF_IDLE;
            if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
            {
                RadioEvents->TxDone();
            }
            break;
        }
        break;
    default:
        break;
    }
}

void Virtual_Radio_OnTimeout_Async(void)
{
    switch (Virtual_Radio.Settings.State)
    {
    case RF_RX_RUNNING:
        if (Virtual_Radio.Settings.Modem == MODEM_FSK)
        {
            Virtual_Radio.Settings.FskPacketHandler.PreambleDetected = false;
            Virtual_Radio.Settings.FskPacketHandler.SyncWordDetected = false;
            Virtual_Radio.Settings.FskPacketHandler.NbBytes = 0;
            Virtual_Radio.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            
            SX1272Write(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                           RF_IRQFLAGS1_PREAMBLEDETECT |
                                           RF_IRQFLAGS1_SYNCADDRESSMATCH);
            SX1272Write(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
            

            if (Virtual_Radio.Settings.Fsk.RxContinuous == true)
            {
                // Continuous mode restart Rx chain
                //SX1272Write(REG_RXCONFIG, SX1272Read(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                TimerStart(&RxTimeoutSyncWord);
            }
            else
            {
                Virtual_Radio.Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWord);
            }
        }
        if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
        {
            RadioEvents->RxTimeout();
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        //
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.

        // BEGIN WORKAROUND

        // Reset the radio
        SX1272Reset();

        // Initialize radio default values
        //SX1272SetOpMode(RF_OPMODE_SLEEP);
        /*
        for (uint8_t i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++)
        {
            SX1272SetModem(RadioRegsInit[i].Modem);
            SX1272Write(RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
        }
        */
        SX1272SetModem(MODEM_FSK);

        // Restore previous network type setting.
        SX1272SetPublicNetwork(Virtual_Radio.Settings.LoRa.PublicNetwork);
        // END WORKAROUND

        Virtual_Radio.Settings.State = RF_IDLE;
        if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
        {
            RadioEvents->TxTimeout();
        }
        break;
    default:
        break;
    }
}