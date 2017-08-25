/*******************************************************************************
* Copyright (c) 2017, Alan Barr
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
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
*******************************************************************************/
#include <ccx_quickstart.h>
#include "ccx_quickstart_defaults.h"

/* Assumption is due to pin constraints only 1 GD0 pin is connected to the
 * device.
 * The second GD0 pin is configured and read via software registers.
 * In theory, this acts as a small speed improvement to work around for SPI
 * Eratta. 
 * This is due to the pin's status can be read from the PKTSTATUS register and
 * doesn't need double checked for corruption. */

/*
 * CCX_QUICKSTART_GDO_CONNECTION
 * Set to 0 (GDO0) or 2 (GDO2) to represent which GDO pin is directly connected
 * to the microcontroller. */

#ifdef CCX_QUICKSTART_USE_SOFT_GDO
#   if CCX_QUICKSTART_GDO_CONNECTION == 0
#       define CCX_PKTSTATUS_GET_GDO_SOFT(_PS)     CCX_PKTSTATUS_GET_GDO2(_PS)
#   elif CCX_QUICKSTART_GDO_CONNECTION = 2
#       define CCX_PKTSTATUS_GET_GDO_SOFT(_PS)     CCX_PKTSTATUS_GET_GDO0(_PS)
#   else 
#       error "CCX_QUICKSTART_GDO_CONNECTION unexpected value"
#   endif
#endif


/* Configuration for the FIFO threshold. 
 * The soft GDO pin will be reflecting the TX FIFO status.
 * The hard GDO pin will be reflecting the RX FIFO status. */
#define CCX_THRESHOLD_BITS                  9
#define CCX_THRESHOLD_TX_FREE               (CCX_TX_FIFO_MAX - 25)
#define CCX_THRESHOLD_RX_FULL               40

/* Asserts when RX FIFO is filled at or above the RX FIFO threshold or the end
 * of packet is reached. 
 * De-asserts when the RX FIFO is empty. */
#define CCX_GDO_HARD_CFG_BITS               1

/* Asserts when the TX FIFO is filled at or above the TX FIFO threshold.
 * De-asserts when the TX FIFO is below the same threshold. */ 
#define CCX_GDO_SOFT_CFG_BITS               2

#define CCX_DEBUG_ASSERTS 1

/* TX and RX FIFOs will have a length and address byte prefixed. This is
 * manually added and will be transparently received. */
#define CCX_FIFO_HEADER_BYTES               2

/* RX FIFO configured to have 2 extra bytes appended containing status
 * information */
#define CCX_RX_FIFO_TRAILER_BYTES           2


static uint8_t ccxQuickstartWriteDefaultSettings(void)
{
#define WRITE_DEFAULT(REG)                                          \
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_##REG,                    \
                                  CCX_QUICKSTART_INITIAL_##REG),    \
                     rtn);

    uint8_t rtn = CCX_ERROR_INTERNAL;

    WRITE_DEFAULT(IOCFG2);
    WRITE_DEFAULT(IOCFG0);  
    WRITE_DEFAULT(PKTLEN);  
    WRITE_DEFAULT(PKTCTRL1);
    WRITE_DEFAULT(PKTCTRL0);
    WRITE_DEFAULT(ADDR);    
    WRITE_DEFAULT(CHANNR);  
    WRITE_DEFAULT(FSCTRL1); 
    WRITE_DEFAULT(FSCTRL0); 
    WRITE_DEFAULT(FREQ2);   
    WRITE_DEFAULT(FREQ1);   
    WRITE_DEFAULT(FREQ0);   
    WRITE_DEFAULT(MDMCFG4); 
    WRITE_DEFAULT(MDMCFG3); 
    WRITE_DEFAULT(MDMCFG2); 
    WRITE_DEFAULT(MDMCFG1); 
    WRITE_DEFAULT(MDMCFG0); 
    WRITE_DEFAULT(DEVIATN); 
    WRITE_DEFAULT(MCSM1);  
    WRITE_DEFAULT(MCSM0);  
    WRITE_DEFAULT(FOCCFG);  
    WRITE_DEFAULT(BSCFG);   
    WRITE_DEFAULT(AGCCTRL2);
    WRITE_DEFAULT(AGCCTRL1);
    WRITE_DEFAULT(AGCCTRL0);
    WRITE_DEFAULT(FREND1);  
    WRITE_DEFAULT(FREND0);  
    WRITE_DEFAULT(FSCAL3);  
    WRITE_DEFAULT(FSCAL2);  
    WRITE_DEFAULT(FSCAL1);  
    WRITE_DEFAULT(FSCAL0);  
    WRITE_DEFAULT(FSTEST);  
    WRITE_DEFAULT(TEST2);   
    WRITE_DEFAULT(TEST1);   
    WRITE_DEFAULT(TEST0);   

    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_MULTIBYTE_PATABLE,
                                  CCX_QUICKSTART_PATABLE_0_DMN),
                     rtn);

    #undef WRITE_DEFAULT

    return CCX_OK;
}


uint8_t ccxQuickstartTransmitVariableLength(uint8_t targetAddress,
                                            const uint8_t *buffer, 
                                            uint8_t length)
{
    uint8_t regRead = 0;
    uint8_t tries = UINT8_MAX;
    uint8_t topUpFifoLength = 0;
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t bufIndex = 0;
    uint8_t previousCount = UINT8_MAX;

    /**************************************************************************/
    /* Assert TX FIFO should be empty and not in underrun before starting */
    /**************************************************************************/

    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES, &regRead), 
                        rtn);

    if (regRead)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_FIFO);
        return rtn;
    }


#if CCX_DEBUG_ASSERTS && 0
    /**************************************************************************/
    /* Assert GDO being used as soft pin reads low to reflect TX FIFO */ 
    /**************************************************************************/

    /* We expect to be using GDOx for FIFO purposes. */
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS, &regRead), 
                     rtn);

    /* Soft GD0 should read low */
    if (CCX_PKTSTATUS_GET_GDO_SOFT(regRead))
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }
#endif
    /**************************************************************************/
    /* Fill TX FIFO with variable length byte and as much data */ 
    /**************************************************************************/

    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_MULTIBYTE_TXFIFO,
                                  length + sizeof(targetAddress)),
                     rtn);

    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_MULTIBYTE_TXFIFO, targetAddress), rtn);

    /* Considering packet length and target addres bytes, see if we can write
     * entire payload to buffer or just until CCX_TX_FIFO_MAX limit */
    topUpFifoLength = CCX_FIFO_HEADER_BYTES + length > CCX_TX_FIFO_MAX ?
                        CCX_TX_FIFO_MAX - (CCX_FIFO_HEADER_BYTES) :
                        length;

    CCX_STATUS_CHECK(ccxWriteBytes(CCX_REG_MULTIBYTE_TXFIFO, 
                                   &buffer[bufIndex],
                                   topUpFifoLength), 
                        rtn);

    bufIndex += topUpFifoLength;
    length   -= topUpFifoLength;

#if 0
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_FIFOTHR, 15), rtn);

   /* We expect to be using GDOx for FIFO purposes. */
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS, &regRead),
                                    rtn);

    if (CCX_PKTSTATUS_GET_GDO_SOFT(regRead) == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_STATE);
        return rtn;
    }
    while(1);

#endif

#if CCX_DEBUG_ASSERTS
    /**************************************************************************/
    /* Debug Assert: TX FIFO should now contain length, address, some data */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &regRead), 
                                          rtn);

    if (regRead != CCX_FIFO_HEADER_BYTES + topUpFifoLength)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }
#endif

#if 0
    /**************************************************************************/
    /* Assert GDO being used as soft pin reads high to reflect TX FIFO 
     * - Need to hit threshold first 
     *   TODO - tests ! */ 
    /**************************************************************************/

    /* We expect to be using GDOx for FIFO purposes. */
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS, &regRead),
                                    rtn);

    if (CCX_PKTSTATUS_GET_GDO_SOFT(regRead) == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }
#endif

    /**************************************************************************/
    /* Wait for CCA to indicate clear */ 
    /**************************************************************************/
    tries = UINT8_MAX;
    while (--tries)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS,
                                               &regRead),
                         rtn);

        if (CCX_PKTSTATUS_GET_CCA(regRead))
        {
            break;
        }

        CCX_STATUS_CHECK(ccxDelay25us(), rtn);
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
        return rtn;
    }

    /**************************************************************************/
    /* Begin transmission */ 
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_STX), rtn);

#if CCX_DEBUG_ASSERTS
    /**************************************************************************/
    /* Assert the transmission has begun */ 
    /**************************************************************************/
    tries = UINT8_MAX;
    while (--tries)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_MARCSTATE,
                                               &regRead), 
                            rtn);

        if (CCX_MARCSTATE_IS_IN_TX(regRead))
        {
            break;
        }
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }
#endif

    /**************************************************************************/
    /* Continually top up TX FIFO until we have given it all the data */ 
    /**************************************************************************/
    while (length && --tries)
    {
    /* Using soft GDO */
#if 0
        /* We expect to be using GDOx for FIFO purposes. */
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS,
                                               &regRead), 
                            rtn);

        /* If TX FIFO not below threshold we'll just wait */
        if (CCX_PKTSTATUS_GET_GDO_SOFT(regRead))
        {
            CCX_STATUS_CHECK(ccxDelay25us(), rtn);
            continue;
        }
        
        topUpFifoLength = length > CCX_THRESHOLD_TX_FREE ?
                          CCX_THRESHOLD_TX_FREE :
                          length;
#else

        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES, 
                                               &regRead),
                         rtn);

        if (CCX_TX_BYTES_IS_UNDERFLOW(regRead))
        {
            CCX_STATUS_LOG(rtn = CCX_ERROR_FIFO);
            return rtn;
        }

        topUpFifoLength = CCX_TX_FIFO_MAX - CCX_TX_BYTES_IN_FIFO(regRead);

#endif

        CCX_STATUS_CHECK(ccxWriteBytes(CCX_REG_MULTIBYTE_TXFIFO, 
                                       &buffer[bufIndex],
                                       topUpFifoLength),
                            rtn);

        bufIndex += topUpFifoLength;
        length   -= topUpFifoLength;

        /* We got some bytes in there, reset tries */
        tries = UINT8_MAX;
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
        return rtn;
    }

    /**************************************************************************/
    /* All data in the FIFO monitor TX FIFO and wait for it drain */
    /**************************************************************************/

    tries = UINT8_MAX;
    while (--tries)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                               &regRead),
                         rtn);

        if (CCX_TX_BYTES_IN_FIFO(regRead) == 0)
        {
            break;
        }

        if (previousCount != regRead)
        {
            tries = UINT8_MAX;
        }

        previousCount = regRead;

        CCX_STATUS_CHECK(ccxDelay25us(), rtn);
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_FIFO);
        return rtn;
    }

    return CCX_OK;
}

/* Assume ccxQuickstartConfigureGdoSoft has been previously called */
uint8_t ccxQuickstartReceiveVariableLength(uint8_t *rxBuffer,
                                           uint8_t *length,
                                           uint8_t *rssi,
                                           uint8_t *lqi)
{
    uint8_t regValues[CCX_FIFO_HEADER_BYTES] = {0};
    uint8_t packetBytesRemaining = 0;
    uint8_t bytesAvailable = 0;
    uint8_t rxBufIndex = 0;
    uint8_t tries = UINT8_MAX;
    uint8_t rtn = CCX_ERROR_INTERNAL;

    /**************************************************************************/
    /* Make sure there is enough data to obtain trailer bytes */
    /**************************************************************************/
    bytesAvailable = 0;
    while ((bytesAvailable < CCX_FIFO_HEADER_BYTES) && tries--)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_RXBYTES, 
                                               &bytesAvailable),
                         rtn);
    }

    /* TODO maybe should reclassify this error - could just be a bad call to
     * this function when no data available */
    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
        return rtn;
    }

    CCX_STATUS_CHECK(ccxReadBytes(CCX_REG_MULTIBYTE_RXFIFO,
                                  regValues,
                                  CCX_FIFO_HEADER_BYTES), rtn);

    /* Advertised length included address byte length */
    packetBytesRemaining = regValues[0] - 1;

    /* Byte regValues[1] is address */

    if (packetBytesRemaining > *length)
    {
        /* Alternatively, could read it, to flush fifos ? */
        CCX_STATUS_LOG(rtn = CCX_ERROR_API);
        return rtn;
    }

    *length = packetBytesRemaining;

    /**************************************************************************/
    /* Now get data for user */
    /**************************************************************************/
    tries = UINT8_MAX;
    while (packetBytesRemaining && --tries)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_RXBYTES, 
                                               &regValues[0]),
                         rtn);

        bytesAvailable = CCX_RX_BYTES_IN_FIFO(regValues[0]);

        if (bytesAvailable == 0)
        {
            CCX_STATUS_CHECK(ccxDelay25us(), rtn);
            continue;
        }

        if (bytesAvailable > packetBytesRemaining)
        {
            bytesAvailable = packetBytesRemaining;
        }

        CCX_STATUS_CHECK(ccxReadBytes(CCX_REG_MULTIBYTE_RXFIFO,
                                      &rxBuffer[rxBufIndex],
                                      bytesAvailable), 
                         rtn);

        rxBufIndex       += bytesAvailable;
        packetBytesRemaining -= bytesAvailable;
        tries = UINT8_MAX;
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
        return rtn;
    }

    /**************************************************************************/
    /* Read Appended RSSI, LQI and CRC */
    /**************************************************************************/
    tries = UINT8_MAX;

    bytesAvailable = 0;
    while ((bytesAvailable < CCX_RX_FIFO_TRAILER_BYTES) && tries--)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_RXBYTES, 
                                               &bytesAvailable),
                         rtn);
    }

    if (tries == 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
        return rtn;
    }

    CCX_STATUS_CHECK(ccxReadBytes(CCX_REG_MULTIBYTE_RXFIFO,
                                  regValues,
                                  CCX_RX_FIFO_TRAILER_BYTES), rtn);

    if (rssi)
    {
        *rssi = regValues[0];
    }

    if (lqi)
    {
        *lqi = CCX_APD_B1_GET_LQI(regValues[1]);
    }

    if (!CCX_APD_B1_IS_CRC_OK(regValues[1]))
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_RX_CRC);
        return rtn;
    }

    if (CCX_RX_BYTES_IS_OVERFLOW(bytesAvailable))
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_FIFO);
        return rtn;
    }

    return CCX_OK;
}


static uint8_t ccxQuickstartInitTweakDefaults(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t gdoCfg = 0;

    /**************************************************************************/
    /* Threshold */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_FIFOTHR, CCX_THRESHOLD_BITS), rtn);

    /**************************************************************************/
    /* Main / Hard GD0 Connection */
    /**************************************************************************/
    gdoCfg = CCX_GDO_HARD_CFG_BITS;

#if CCX_QUICKSTART_GDO_CONNECTION == 0
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG0, gdoCfg), rtn);
#else
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG2, gdoCfg), rtn);
#endif

    /**************************************************************************/
    /* Soft GD0 Connection */
    /**************************************************************************/
#ifdef CCX_QUICKSTART_USE_SOFT_GDO
    gdoCfg = CCX_GDO_SOFT_CFG_BITS;
#if CCX_QUICKSTART_GDO_CONNECTION == 0
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG2, gdoCfg), rtn);
#else
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG0, gdoCfg), rtn);
#endif
#endif /* CCX_QUICKSTART_USE_SOFT_GDO */

    return CCX_OK;
}

uint8_t ccxQuickstartInit(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;

    CCX_STATUS_CHECK(ccxInit(), rtn);

    CCX_STATUS_CHECK(ccxQuickstartWriteDefaultSettings(), rtn);

    CCX_STATUS_CHECK(ccxQuickstartInitTweakDefaults(), rtn);

    return CCX_OK;
}

uint8_t ccxQuickstartShutdown(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;

    CCX_STATUS_CHECK(ccxShutdown(), rtn);

    return CCX_OK;
}

