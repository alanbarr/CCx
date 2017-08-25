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
#include <ccx_api.h>

/** 
 * @file
 *
 * @brief Optional functionality for convience. See @ref groupQuickstart.
 */

/******************************************************************************/
/**
 * @defgroup groupQuickstart Quick Start API
 *
 * @brief Additional, higher level API functions. 
 *
 * This functions are built on top of the basic functions contained within @ref
 * groupCCx. They make more assumptions / stipulations on the CCxxxx
 * configuration which may not be appropriate for all users.
 *
 * They will configure the radio's registers with a setup provided by TI in
 * slaa325a.
 *
 * Additionally, they expect either the CC1101's GD0 or GD2 pin to be used as an
 * interrupt source in order to receive packets.
 */
/**@{*/
/******************************************************************************/


/**
 * @brief Initiate the transmission of a variable length packet.
 *
 * This function will block until the message is transmitted.
 *
 * @param [in] targetAddress 
 * The address of the intended recipient of this message. This is set within
 * the device's @ref CCX_REG_ADDR register.
 *
 * @param [in] buffer 
 * The data buffer to transmit.
 *
 * @param [in] length 
 * The length of the data in @p buffer.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxQuickstartTransmitVariableLength(uint8_t targetAddress,
                                            const uint8_t *buffer, 
                                            uint8_t length);

/**
 * @brief Initiate retrieving a variable length packet from the RX FIFO.
 *
 * This function will block until the entire packet is received.
 *
 * @param [out] rxBuffer 
 * An array to be populated with the received data. This must be at least as
 * long as the received message.
 *
 * @param [in,out] length
 * The length of the @p rxBuffer to be populated. On a successful call, this
 * variable will contain the length of the received data which populates @p
 * rxBuffer.
 *
 * @param [out] rssi
 * Optional parameter. If not @p NULL, this will be set to the RSSI value
 * appended to the received message.
 *
 * @param [out] lqi
 * Optional parameter. If not @p NULL, this will be set to the LQI value
 * appended to the received message.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxQuickstartReceiveVariableLength(uint8_t *rxBuffer,
                                           uint8_t *length,
                                           uint8_t *rssi,
                                           uint8_t *lqi);

/**
 * @brief Initialises the QuickStart module.
 *
 * This funciton initalises lower layer modules, including the SPI layer.
 * Additionally this function will configiure the CCxxxx device with preset
 * configuration. See @ref groupQuickstart for more information.
 *
 * @note This calls @ref ccxReset internally.
 *
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxQuickstartInit(void);

/**
 * @brief
 * @return
 * @return A status code from @ref CcxStatus.
 */
uint8_t ccxQuickstartShutdown(void);

/******************************************************************************/
/**@}*/
/******************************************************************************/
