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
#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>
#include "ccx_api.h"

/* Asserts when the TX FIFO is filled at or above the TX FIFO threshold.
 * De-asserts when the TX FIFO is below the same threshold. */ 
#define TEST_GDO_OUTPUT_TX_THRESHOLD_CFG     2

/* Configuration for the FIFO threshold. */
#define TEST_THRESHOLD_CFG                   9

/* Number of bytes in TX FIFO at which threshold is crossed */
#define TEST_THRESHOLD_TX_BYTES              25

/* Interrupt for GDx pin should be disabled for this test. */

static uint8_t writeToTxFifo(uint8_t length)
{
    const char *txData =
        "I do not kill with my gun; "
        "he who kills with his gun has forgotten the face of his father. "
        "I kill with my heart.";

    return ccxWriteBytes(CCX_REG_MULTIBYTE_TXFIFO, 
                         (const uint8_t *)txData,
                         length);
}

static uint8_t assertGdo(uint8_t expected)
{
    uint8_t readByte = 0;
    uint8_t rtn = CCX_ERROR_ASSERT;

    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_PKTSTATUS, &readByte), 
                     rtn);

    if (CCX_PKTSTATUS_GET_GDO0(readByte) != expected ||
        CCX_PKTSTATUS_GET_GDO2(readByte) != expected)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    return CCX_OK;
}

static uint8_t testCheckGdoNearThreshold(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t readByte = 0;

    /**************************************************************************/
    /* Write Bytes to TX FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(writeToTxFifo(TEST_THRESHOLD_TX_BYTES-1), rtn);

    /**************************************************************************/
    /* Check bytes in TX FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != TEST_THRESHOLD_TX_BYTES-1)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GD0 and GD2 reads low */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(0), rtn);

    /**************************************************************************/
    /* Write more Bytes to TX FIFO to surpass threshold */
    /**************************************************************************/
    CCX_STATUS_CHECK(writeToTxFifo(1), rtn);

    /**************************************************************************/
    /* Check bytes in TX FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                           &readByte), 
                                           rtn);

    if (readByte != TEST_THRESHOLD_TX_BYTES)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GD0 and GD2 reads high */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(1), rtn);

    /**************************************************************************/
    /* Flush FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SFTX), rtn);

    /**************************************************************************/
    /* Check no bytes in TX FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GDO low */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(0), rtn);

    return rtn;
}

static uint8_t testCheckStateChanges(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t readByte = 0;

    /**************************************************************************/
    /* Write bytes to TX FIFO, passing threshold */
    /**************************************************************************/
    CCX_STATUS_CHECK(writeToTxFifo(TEST_THRESHOLD_TX_BYTES), rtn);

    /**************************************************************************/
    /* Check GDO high */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(1), rtn);

    /**************************************************************************/
    /* Move to Idle */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SIDLE), rtn);

    /**************************************************************************/
    /* Check in idle */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_MARCSTATE,
                                           &readByte), 
                            rtn);

    if (CCX_MARCSTATE_GET_STATE(readByte) != CCX_MARCSTATE_IDLE)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GDO high */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(1), rtn);

    /**************************************************************************/
    /* Move to Receive */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SRX), rtn);

    /**************************************************************************/
    /* Check in receive */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_MARCSTATE,
                                           &readByte), 
                            rtn);

    if (CCX_MARCSTATE_GET_STATE(readByte) != CCX_MARCSTATE_RX)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Read TX FIFO size */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != TEST_THRESHOLD_TX_BYTES)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Flush FIFO */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SFTX), rtn);

    /**************************************************************************/
    /* Check TX FIFO size */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GDO low */
    /**************************************************************************/

    return rtn;
}

static uint8_t testCheckReset(void)
{
    uint8_t readByte = 0;
    uint8_t rtn = CCX_ERROR_INTERNAL;

    /**************************************************************************/
    /* Write bytes to TX FIFO to surpass threshold */
    /**************************************************************************/
    CCX_STATUS_CHECK(writeToTxFifo(64), rtn);

    /**************************************************************************/
    /* Check GDO high */
    /**************************************************************************/
    CCX_STATUS_CHECK(assertGdo(0), rtn);

    /**************************************************************************/
    /* Reset Device */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SRES), rtn);

    /**************************************************************************/
    /* TX FIFO Empty */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_TXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* RX FIFO Empty */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_RXBYTES,
                                          &readByte), 
                                          rtn);

    if (readByte != 0)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Check GDO configuration no longer matches what we configured */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadByte(CCX_REG_IOCFG0, &readByte), rtn);

    if (readByte == TEST_GDO_OUTPUT_TX_THRESHOLD_CFG)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    CCX_STATUS_CHECK(ccxReadByte(CCX_REG_IOCFG2, &readByte), rtn);

    if (readByte == TEST_GDO_OUTPUT_TX_THRESHOLD_CFG)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    return rtn;
}

static uint8_t testSetupThreshold(void)
{
    uint8_t readByte = 0;
    uint8_t rtn = CCX_ERROR_ASSERT;

    /**************************************************************************/
    /* Set FIFO Threshold and validate write */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_FIFOTHR, TEST_THRESHOLD_CFG), rtn);

    CCX_STATUS_CHECK(ccxReadByte(CCX_REG_FIFOTHR, &readByte), rtn);

    if (readByte != TEST_THRESHOLD_CFG)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Set GD0 and GD2 to indicate TX threshold */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG0, 
                                  TEST_GDO_OUTPUT_TX_THRESHOLD_CFG), 
                     rtn);

    CCX_STATUS_CHECK(ccxWriteByte(CCX_REG_IOCFG2,
                                  TEST_GDO_OUTPUT_TX_THRESHOLD_CFG), 
                     rtn);

    /**************************************************************************/
    /* Validate registers were written */
    /**************************************************************************/
    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_IOCFG0, &readByte), rtn);

    if (readByte != TEST_GDO_OUTPUT_TX_THRESHOLD_CFG)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_IOCFG2, &readByte), rtn);

    if (readByte != TEST_GDO_OUTPUT_TX_THRESHOLD_CFG)
    {
        CCX_STATUS_LOG(rtn = CCX_ERROR_ASSERT);
        return rtn;
    }

    /**************************************************************************/
    /* Ensure GDO lines are low */
    /**************************************************************************/
    assertGdo(0);

    return rtn;
}

uint8_t ccxRunTests(void)
{
    uint8_t rtn = CCX_ERROR_INTERNAL;
    uint8_t iterations = 0;

    for (iterations = 0; iterations < 5; iterations++)
    {
        CCX_STATUS_CHECK(ccxInit(), rtn);

        CCX_STATUS_CHECK(testSetupThreshold(), rtn);

        CCX_STATUS_CHECK(testCheckGdoNearThreshold(), rtn);

        CCX_STATUS_CHECK(testCheckStateChanges(), rtn);

        CCX_STATUS_CHECK(testCheckReset(), rtn);

        CCX_STATUS_CHECK(ccxShutdown(), rtn);
    }

    return rtn;
}

