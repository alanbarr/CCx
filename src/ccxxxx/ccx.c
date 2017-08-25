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

/**
 * @file
 *
 * @details Contains basic functionality for controlling the TI ccx chip.
 */

#include <ccx_api.h>
#include "ccx_spi.h"

#define ccxCommunicationsStop()  ccxSpiPinChipSelectSet(1)

static uint8_t ccxCommunicationsStart(void)
{
    uint8_t tries = UINT8_MAX;
    uint8_t slaveOut = 1;
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t cleanupRtn = CCX_ERROR_INTERNAL;

    CCX_STATUS_CHECK(ccxSpiPinChipSelectSet(0), rtn);

    while (tries)
    {
        CCX_STATUS_CHECK_CLEANUP(ccxSpiPinSlaveOutRead(&slaveOut), 
                                 rtn,
                                 goto error_cleanup);

        if (slaveOut == 0)
        {
            return CCX_OK;
        }

        CCX_STATUS_CHECK_CLEANUP(ccxSpiDelay25us(),
                                 rtn,
                                 goto error_cleanup);
        tries--;
    }

    CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);

    /* Only get here in case of error. Log new errors but don't lose previous
     * error */
error_cleanup:
    CCX_STATUS_CHECK(ccxSpiPinChipSelectSet(1), cleanupRtn);

    return rtn;
}

/* writeBufferLength == 0 and writeBuffer == NULL valid */
uint8_t ccxWriteBytes(uint8_t address,
                      const uint8_t *writeBuffer,
                      uint8_t writeBufferLength)
{
    uint8_t rtn = CCX_OK;
    uint8_t cleanupRtn = CCX_OK;

    if (writeBufferLength > 1)
    {
        address |= CCX_HEADER_BURST;
    }

    CCX_STATUS_CHECK(ccxCommunicationsStart(), rtn);

    CCX_STATUS_CHECK_CLEANUP(ccxSpiTransaction(address,
                                               writeBuffer, writeBufferLength,
                                               NULL, 0),
                             rtn, 
                             goto cleanup);
cleanup:
    CCX_STATUS_CHECK(ccxCommunicationsStop(), cleanupRtn);

    return rtn == CCX_OK ? cleanupRtn : rtn;
}

uint8_t ccxWriteByte(uint8_t address, uint8_t value)
{
    return ccxWriteBytes(address, &value, 1);
}

uint8_t ccxStrobe(uint8_t strobeAddress)
{
    return ccxWriteBytes(strobeAddress, NULL, 0);
}

uint8_t ccxReadBytes(uint8_t startAddress,
                     uint8_t *readBuffer,
                     uint8_t readBufferLength)
{
    uint8_t rtn = CCX_OK;
    uint8_t cleanupStatus = CCX_OK;

    if (readBuffer == NULL)
    {
        CCX_STATUS_CHECK(CCX_ERROR_API, rtn);
    }
    
    startAddress |= CCX_HEADER_READ;

    if (readBufferLength > 1)
    {
        startAddress |= CCX_HEADER_BURST;
    }

    CCX_STATUS_CHECK(ccxCommunicationsStart(), rtn);

    CCX_STATUS_CHECK_CLEANUP(ccxSpiTransaction(startAddress,
                                               NULL, 0,
                                               readBuffer, readBufferLength),
                             rtn,
                             goto cleanup);

cleanup:
    CCX_STATUS_CHECK(ccxCommunicationsStop(),
                        cleanupStatus);

    return rtn;
}

uint8_t ccxReadByte(uint8_t address, uint8_t *value)
{
    return ccxReadBytes(address, value, 1);
}

uint8_t ccxReadStatusRegister(uint8_t baseAddress, 
                              uint8_t *readByte)
{
    uint8_t tries = UINT8_MAX;
    uint8_t lastRead = 0;
    uint8_t rtn = CCX_ERROR_INTERNAL;

    /* Handling for "SPI Read Syncronization issue" Eratta */
    switch (baseAddress)
    {
        /* Static or single bit registers not affected */
        case CCX_REG_STATUS_VERSION:
        case CCX_REG_STATUS_PARTNUM:
        case CCX_REG_STATUS_PKTSTATUS:

        /* Not affected as should only be read after reception / transmission */
        case CCX_REG_STATUS_VCO_VC_DAC:
        case CCX_REG_STATUS_LQI:
        case CCX_REG_STATUS_FREQEST:
            tries = 0;
            break;

        default:
            tries = UINT8_MAX;
            break;
    }
    
    /* Status registers read only. Burst bit has different meaning - forces new
     * address space */
    baseAddress |= CCX_HEADER_READ | CCX_HEADER_BURST;
    
    CCX_STATUS_CHECK(ccxReadByte(baseAddress, readByte), rtn);

    if (tries == 0)
    {
        return CCX_OK;
    }

    /* Register in question affected by eratta. Keep reading until we get 2
     * sequential values the same. */
    while (tries--)
    {
        lastRead = *readByte;

        CCX_STATUS_CHECK(ccxReadByte(baseAddress, readByte), rtn);

        if (lastRead == *readByte)
        {
            return CCX_OK;
        }
    }

    CCX_STATUS_LOG(rtn = CCX_ERROR_TIMEOUT);
    return rtn;
}

uint8_t ccxDelay25us(void)
{
    return ccxSpiDelay25us();
}

uint8_t ccxReset(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;

    CCX_STATUS_CHECK(ccxSpiPinChipSelectSet(1), rtn);

    CCX_STATUS_CHECK_CLEANUP(ccxDelay25us(), rtn, goto cleanup);

    CCX_STATUS_CHECK(ccxSpiPinChipSelectSet(0), rtn);

    CCX_STATUS_CHECK_CLEANUP(ccxDelay25us(), rtn, goto cleanup);

    CCX_STATUS_CHECK_CLEANUP(ccxSpiPinChipSelectSet(1), rtn, goto cleanup);

    CCX_STATUS_CHECK_CLEANUP(ccxDelay25us(), rtn, goto cleanup);
    CCX_STATUS_CHECK_CLEANUP(ccxDelay25us(), rtn, goto cleanup);

    CCX_STATUS_CHECK_CLEANUP(ccxStrobe(CCX_REG_STROBE_SRES), rtn, goto cleanup);

cleanup:
    CCX_STATUS_CHECK(ccxSpiPinChipSelectSet(1), rtn);

    return CCX_OK;
}

uint8_t ccxInit(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;

    CCX_STATUS_CHECK(ccxSpiInit(), rtn);

    CCX_STATUS_CHECK(ccxReset(), rtn);

    return CCX_OK;
}

uint8_t ccxShutdown(void)
{
    /* Put the device into sleep mode ? */
    return ccxSpiShutdown();
}
