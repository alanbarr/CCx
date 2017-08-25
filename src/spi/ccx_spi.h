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

#ifndef __CC1101_SPI_H__
#define __CC1101_SPI_H__

#include <ccx_api.h>

/**
 * @file
 * @brief Provides the function prototypes for interfacing with the CCxxxx over
 * SPI. See @ref groupSpiHw.
 * 
 */

/******************************************************************************/
/**
 * @defgroup groupSpiHw SPI Hardware Implementation
 * @brief Required MCU-to-CCxxxx functions.
 *
 * If porting this driver to new hardware, the user must create the necessary
 * function definitions and implement them as described. 
 */
/**@{*/
/******************************************************************************/
/**
 * @brief Initialise the SPI hardware
 *
 * This function will be called when the CC1101 module is
 * initialised. The implementation is expected to preform any necessary
 * configuration, including:
 *
 * - Set Chip Select (CS) line to output, initial value high. Due to the nature
 *   of the CC1101, this line is should be controlled by the driver,
 *   independently of the SPI hardware.
 * - Prepare Slave In (SI), Slave Out (SO) and Clock (CLK) pins as per hardware
 *   requirements.
 * - Configure the pin attached to the CC1101 GDO pin to input / interrupt.
 *   (Optional, but likely recommended depending on your implementation).
 *
 * @section sectionSpiConfiguration SPI Configuration
 *
 * - Clock Frequency: The clock can run at 10 MHz if 100 ns delay added between
 *   each octet. Otherwise single access should be limited to 9 MHz, and burst
 *   access 6.5 MHz.
 * - Clock phase: Data centered on first rising clock edge.
 * - Clock polarity: Low when idle
 *
 * @note Refer to Texas Instruments Design Note DN503 for more information on
 * interfacing over SPI.
 * 
 */
uint8_t ccxSpiInit(void);

/**
 * @brief Shutdown the SPI hardware
 *
 * This will be called when the CC1101 module is shutdown. Depending
 * on the user implementation of the above driver it may never be called.
 */
uint8_t ccxSpiShutdown(void);

/**
 * @brief Set the state of the Chip Select (CS) line.
 * @param [in] setValue 
 *  The desired state of the CS line.
 *      - "1" for a high output (to not select the CC1101)
 *      - "0" for a low output (to select the CC1101)
 */
uint8_t ccxSpiPinChipSelectSet(uint8_t setValue);

/**
 * @brief Read the current state of the Slave Out (SO) line.
 *
 * @param [out] currentValue 
 *  The current state of the SO line:
 *      - "1" the line is currently driven high by CC1101
 *      - "0" the line is currently driven low by CC1101
 *
 * The SO line is used to indicate the readiness of the CC1101 to
 * receive commands after the Chip Select line has been used to indicate
 * interaction is desired. It is assumed the hardware implementation can read
 * the state of this line when not immediately transmitting data over SPI.
 */
uint8_t ccxSpiPinSlaveOutRead(uint8_t *currentValue);

/**
 * @brief Start a SPI transaction to the CC1101, either read or write.
 *
 * The transaction is expected to be blocking.
 *
 * @param [in] address
 * The register address. This should be always be sent to the device as the
 * first byte.  
 *
 * @param [in] writeBytes
 * Array of bytes to write. Length of the array is provided as
 * @p writeLength. 
 * This can be @p NULL.
 *
 * @param [in] writeLength
 * The length of the @p writeBytes array. This can be 0.
 *
 * @param [out] readBytes
 * Array to store read data into. Length of this array is provided as @p
 * readLength.  
 * This can be @p NULL.
 *
 * @param [out] readLength
 * Length of the array @p readBytes. This can be 0.
 *
 * @section sectionExampleTransfers Example Transfers
 *
 * The following depecits some example transfers:
 * @code
 * writeLength == 0 and readLength == 0 
 * Write:   | address           | 
 * Read:    | status            | 
 *
 * writeLength == 1 and readLength == 0
 * Write:   | address           | 
 * Read:    | status            | 
 *
 * writeLength == 3 and readLength == 0
 * Write:   | address           | write byte 0  | write byte 1  | write byte 2
 * Read:    | status            | status        | status        | status        
 *
 * readLength == 1 and writeLength == 0
 * Write:   | address           |   - 
 * Read:    | status            | read byte 0 
 *
 * readLength == 3 and writeLength == 0
 * Write:   | address           |   -           |   -           |   -
 * Read:    | status            | read byte 0   | read byte 1   | read byte 2
 * @endcode
 *
 */
uint8_t ccxSpiTransaction(uint8_t address,
                          const uint8_t *writeBytes,
                          uint8_t writeLength,
                          uint8_t *readBytes,
                          uint8_t readLength);

/**
 * @brief Provide a reliable, 25 us delay.
 * 
 * Exact timing is not important, and assumed this will be implemented as a busy
 * wait. 
 */
uint8_t ccxSpiDelay25us(void);

/******************************************************************************/
/** @}*/
/******************************************************************************/

#endif
