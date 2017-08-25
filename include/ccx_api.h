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
 * @brief CCX Basic API definitions. See @ref groupCCx.
 *
 */

#ifndef __CCX_API_H__
#define __CCX_API_H__

#include <cc1100_cc2500.h>
#include <stddef.h>
#include <stdint.h>
/******************************************************************************/
/**
 * @defgroup groupCCx Basic CCxxxx API
 * @brief Basic API to interface with the CCxxxx chip.
 *
 * Note: There must be a SPI port for hardware in use. See @ref groupSpiHw for
 * more information on the functions that must be defined.
 */
/**@{*/
/******************************************************************************/


#if CCX_LOG_ERRORS || defined(__DOXYGEN__)
    /** 
     * @brief Macro called to log errors.
     *
     * Will provide file name and line number to user defined function 
     * @ref ccxErrorLog.
     *
     * The behaviour of this macro depends on compile time flags.
     *
     * @param _STATUS Encountered error from @ref CcxStatus.
     */
#   define CCX_STATUS_LOG(_STATUS)  _CCX_STATUS_LOG(_STATUS, __FILE__, __LINE__)
#else
    /* Error logging not desired, do nothing */
#   define CCX_STATUS_LOG(_STATUS) 
#endif

/** 
 * @brief Hacky middle-man to get string line number.
 *
 * See @ref CCX_STATUS_LOG.
 */
#define __CCX_STATUS_LOG(_STATUS, F, L)  ccxErrorLog(_STATUS, F, #L)

/** 
 * @brief Hacky middle-man to get string line number.
 *
 * See @ref CCX_STATUS_LOG.
 */
#define _CCX_STATUS_LOG(_STATUS, F, L)  __CCX_STATUS_LOG(_STATUS, F, L)

/**
 * @brief Convenience error handling function. Returns from current function
 * with evaluated error code.
 *
 * @param _STATEMENT Statement to evaluate. Assumed to return a @ref CcxStatus
 *                   error code.
 * @param _STATUS Local variable to hold @ref CcxStatus code.
 */
#define CCX_STATUS_CHECK(_STATEMENT, _STATUS)                       \
do {                                                                \
    _STATUS = (_STATEMENT);                                         \
    if ((_STATUS) != CCX_OK)                                        \
    {                                                               \
        CCX_STATUS_LOG(_STATUS);                                    \
        return _STATUS;                                             \
    }                                                               \
} while (0);

/**
 * @brief Convenience error handling function. In the event of an error attempts 
 * cleanup.
 *
 * @param _STATEMENT Statement to evaluate. Assumed to return a @ref CcxStatus
 *                   error code.
 * @param _STATUS Local variable to hold @ref CcxStatus code.
 * @param _CLEANUP Command to execute on error. Typically expected to be a "goto
 *                 label".
 */
#define CCX_STATUS_CHECK_CLEANUP(_STATEMENT, _STATUS, _CLEANUP)     \
do {                                                                \
    _STATUS = (_STATEMENT);                                         \
    if ((_STATUS) != CCX_OK)                                        \
    {                                                               \
        CCX_STATUS_LOG(_STATUS);                                    \
        _CLEANUP;                                                   \
    }                                                               \
} while (0);

/**
 * @brief List of possible error return codes.
 *
 * All API functions return a CcxStatus value. Any value other than @p CCX_OK
 * indicates an error occurred and function call was unsuccessful. Additional
 * cleanup may be required by the user's application, e.g. flushing the FIFO.
 */
typedef enum {
    CCX_OK,              /**< @brief No error, operational as expected. */
    CCX_ERROR_API,       /**< @brief Bad value passed by API. */
    CCX_ERROR_INTERNAL,  /**< @brief Internal, unexpected error detected. */
    CCX_ERROR_TIMEOUT,   /**< @brief Communications timed out. */
    CCX_ERROR_SPI,       /**< @brief SPI error. */
    CCX_ERROR_RX_CRC,    /**< @brief Received CRC was incorrect. */
    CCX_ERROR_STATE,     /**< @brief Device was in an unexpected state. */
    CCX_ERROR_FIFO,      /**< @brief One or more of the FIFOs are in error. */
    CCX_ERROR_ASSERT,    /**< @brief Sanity assert failed. */
} CcxStatus;


/**
 * @brief Optional error logging function.
 *
 * If error logging is enabled, this function must be provided by the user.
 *
 * @param status Error encountered. Will be one of @ref CcxStatus.
 * @param file Name of the file error was logged.
 * @param line Line number where the error was logged.
 */
void ccxErrorLog(uint8_t status, char *file, char *line);


/** 
 * @brief Write a single byte (octet) to the CCxxxx.
 *
 * @param address The register address to write to. See @ref cc1100_cc2500.h.
 * @param value The byte to write to @p address.
 */
uint8_t ccxWriteByte(uint8_t address, uint8_t value);

/**
 * @brief Write multiple bytes (oxtets) to the CCxxxx.
 *
 * @param [in] address 
 * The register address to write to. See @ref cc1100_cc2500.h.
 *
 * @param [in] writeBuffer 
 * A continuous array of bytes to write to the device.
 * The behaviour of a sequential write varies depending on the @p address. See
 * the CCxxxx device data sheet for more information.
 *
 * @param [in] writeBufferLength 
 * The number of bytes to be written (the length of @p writeBuffer ).
 */
uint8_t ccxWriteBytes(uint8_t address,
                      const uint8_t *writeBuffer,
                      uint8_t writeBufferLength);

/**
 * @brief Send a strobe to the CCxxx.
 *
 * Strobes are single byte commands. 
 *
 * @param strobeAddress The strobe command to send. This should be one of
 * @ref groupStrobeCommands.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxStrobe(uint8_t strobeAddress);

/** 
 * @brief Read a single byte from specified register.
 * @param [in] address Register address to read. See @ref cc1100_cc2500.h.
 * @param [out] readByte The register contents.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxReadByte(uint8_t address, uint8_t *readByte);

/** 
 * @brief Read multiple bytes beginning at the specified register.
 *
 * @param [in] startAddress 
 * Register address to start read operation on. Certain CCxxxx registers behave
 * differently on a burst read operation. Consult the data sheet for more
 * information. See @ref cc1100_cc2500.h.  
 *
 * @param [out] readBuffer 
 * The buffer to be populated by the read operation.
 *
 * @param [in] readBufferLength
 * The length of @p readBuffer / the desired number of bytes to read.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxReadBytes(uint8_t startAddress,
                     uint8_t *readBuffer,
                     uint8_t readBufferLength);

/**
 * @brief Read a single byte from a status register.
 *
 * @param [in] address 
 * The status register address to read. 
 * This function will ensure the read bit and burst bit of @p address is set
 * appropriately.
 * See @ref groupStatusRegisters.
 *
 * @param [out] readStatus The contents of the status register after read is
 * complete.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxReadStatusRegister(uint8_t address, 
                              uint8_t *readStatus);

/**
 * @brief Initiate a busy, blocking wait of 25 us.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxDelay25us(void);

/**
 * @brief Initiate a software reset of the radio.
 *
 * This includes the steps required for the manual power-up sequence as
 * documented in the data sheet.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxReset(void);

/**
 * @brief Initialise the ccx module. This must be called prior to using any
 * other function in this library.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxInit(void);

/**
 * @brief Shutdown the module. 
 * Can optionally be called when finished with using the module.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxShutdown(void);


/** 
 * @brief Run basic sanity tests to ensure communications between MCU and CCxxxx
 * module are operating as expected.
 *
 * @return A status code from @ref CcxStatus. @p CCX_OK indicates all tests
 * passed.
 */
uint8_t ccxRunTests(void);

/******************************************************************************/
/** @}*/
/******************************************************************************/


#endif
