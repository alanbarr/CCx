/******************************************************************************
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
 * @brief Contains CCxxxx register and bitfield values 
 * See @ref groupNormalRegisters,
 *     @ref groupStatusRegisters,
 *     @ref groupMultibyteRegisters,
 *     @ref groupStrobeCommands
 *
 *
 * Values obtained from datasheet "CC1101 Low-Power Sub-1 GHz RF
 * Transceiver", "TI", "SWRS061".
 */

#ifndef __CCX_REGISTERS_H__
#define __CCX_REGISTERS_H__


/******************************************************************************/
/**
 * @defgroup groupNormalRegisters Registers: Normal
 * @brief Address space for the CCxxxx "normal" registers.
 */
/**@{*/
/******************************************************************************/

#define CCX_REG_IOCFG2      0x00 /**<@brief GDO2 output pin configuration.  */
#define CCX_REG_IOCFG1      0x01 /**<@brief GDO1 output pin configuration   */
#define CCX_REG_IOCFG0      0x02 /**<@brief GDO0 output pin configuration   */
#define CCX_REG_FIFOTHR     0x03 /**<@brief RX FIFO and TX FIFO thresholds  */
#define CCX_REG_SYNC1       0x04 /**<@brief Sync word, high byte            */
#define CCX_REG_SYNC0       0x05 /**<@brief Sync word, low byte             */
#define CCX_REG_PKTLEN      0x06 /**<@brief Packet length                   */
#define CCX_REG_PKTCTRL1    0x07 /**<@brief Packet automation control       */
#define CCX_REG_PKTCTRL0    0x08 /**<@brief Packet automation control       */
#define CCX_REG_ADDR        0x09 /**<@brief Device address                  */
#define CCX_REG_CHANNR      0x0A /**<@brief Channel number                  */
#define CCX_REG_FSCTRL1     0x0B /**<@brief Frequency synthesizer control   */
#define CCX_REG_FSCTRL0     0x0C /**<@brief Frequency synthesizer control   */
#define CCX_REG_FREQ2       0x0D /**<@brief Frequency control word, high byte*/
#define CCX_REG_FREQ1       0x0E /**<@brief Frequency control word, middle byte*/
#define CCX_REG_FREQ0       0x0F /**<@brief Frequency control word, low byte*/
#define CCX_REG_MDMCFG4     0x10 /**<@brief Modem configuration             */
#define CCX_REG_MDMCFG3     0x11 /**<@brief Modem configuration             */
#define CCX_REG_MDMCFG2     0x12 /**<@brief Modem configuration             */
#define CCX_REG_MDMCFG1     0x13 /**<@brief Modem configuration             */
#define CCX_REG_MDMCFG0     0x14 /**<@brief Modem configuration             */
#define CCX_REG_DEVIATN     0x15 /**<@brief Modem deviation setting         */
#define CCX_REG_MCSM2       0x16 /**<@brief Main Radio Control State Machine configuration*/
#define CCX_REG_MCSM1       0x17 /**<@brief Main Radio Control State Machine configuration*/
#define CCX_REG_MCSM0       0x18 /**<@brief Main Radio Control State Machine configuration*/
#define CCX_REG_FOCCFG      0x19 /**<@brief Frequency Offset Compensation configuration*/
#define CCX_REG_BSCFG       0x1A /**<@brief Bit Synchronization configuration*/
#define CCX_REG_AGCCTRL2    0x1B /**<@brief AGC control                     */
#define CCX_REG_AGCCTRL1    0x1C /**<@brief AGC control                     */
#define CCX_REG_AGCCTRL0    0x1D /**<@brief AGC control                     */
#define CCX_REG_WOREVT1     0x1E /**<@brief High byte Event 0 timeout       */
#define CCX_REG_WOREVT0     0x1F /**<@brief Low byte Event 0 timeout        */
#define CCX_REG_WORCTRL     0x20 /**<@brief Wake On Radio control           */
#define CCX_REG_FREND1      0x21 /**<@brief Front end RX configuration      */
#define CCX_REG_FREND0      0x22 /**<@brief Front end TX configuration      */
#define CCX_REG_FSCAL3      0x23 /**<@brief Frequency synthesizer calibration*/
#define CCX_REG_FSCAL2      0x24 /**<@brief Frequency synthesizer calibration*/
#define CCX_REG_FSCAL1      0x25 /**<@brief Frequency synthesizer calibration*/
#define CCX_REG_FSCAL0      0x26 /**<@brief Frequency synthesizer calibration*/
#define CCX_REG_RCCTRL1     0x27 /**<@brief RC oscillator configuration     */
#define CCX_REG_RCCTRL0     0x28 /**<@brief RC oscillator configuration     */
#define CCX_REG_FSTEST      0x29 /**<@brief Frequency synthesizer calibration control*/
#define CCX_REG_PTEST       0x2A /**<@brief Production test                 */
#define CCX_REG_AGCTEST     0x2B /**<@brief AGC test                        */
#define CCX_REG_TEST2       0x2C /**<@brief Various test settings           */
#define CCX_REG_TEST1       0x2D /**<@brief Various test settings           */
#define CCX_REG_TEST0       0x2E /**<@brief Various test settings           */

/**@}*/

/******************************************************************************/
/**
 * @defgroup groupStrobeCommands Registers: Strobe Command 
 * @brief Strobe command addresses
 */
/**@{*/
/******************************************************************************/

#define CCX_REG_STROBE_SRES      0x30 /**<@brief Reset chip */
#define CCX_REG_STROBE_SFSTXON   0x31 /**<@brief Enable and calibrate frequency synthesizer. */
#define CCX_REG_STROBE_SXOFF     0x32 /**<@brief Turn off crystal oscillator.  */
#define CCX_REG_STROBE_SCAL      0x33 /**<@brief Calibrate frequency synthesizer and turn it off. */
#define CCX_REG_STROBE_SRX       0x34 /**<@brief Enable RX. */
#define CCX_REG_STROBE_STX       0x35 /**<@brief Enable TX. */
#define CCX_REG_STROBE_SIDLE     0x36 /**<@brief Exit RX/TX. */
#define CCX_REG_STROBE_SWOR      0x38 /**<@brief Start automatic RX polling sequence. */
#define CCX_REG_STROBE_SPWD      0x39 /**<@brief Enter power down mode when CSn goes high. */
#define CCX_REG_STROBE_SFRX      0x3A /**<@brief Flush the RX FIFO buffer. */
#define CCX_REG_STROBE_SFTX      0x3B /**<@brief Flush the TX FIFO buffer. */
#define CCX_REG_STROBE_SWORRST   0x3C /**<@brief Reset real time clock to Event1 value. */
#define CCX_REG_STROBE_SNOP      0x3D /**<@brief No operation. */

/**@}*/

/******************************************************************************/
/**
 * @defgroup groupStatusRegisters Registers: Status
 * @brief Status Register Addresses
 */
/**@{*/
/******************************************************************************/
#define CCX_REG_STATUS_PARTNUM      0x30 /**<@brief Part number */
#define CCX_REG_STATUS_VERSION      0x31 /**<@brief Current version number*/
#define CCX_REG_STATUS_FREQEST      0x32 /**<@brief Frequency offset estimate*/
#define CCX_REG_STATUS_LQI          0x33 /**<@brief Demodulator estimate for link quality*/
#define CCX_REG_STATUS_RSSI         0x34 /**<@brief Received signal strength indication*/
#define CCX_REG_STATUS_MARCSTATE    0x35 /**<@brief Control state machine state*/
#define CCX_REG_STATUS_WORTIME1     0x36 /**<@brief High byte of WOR timer*/
#define CCX_REG_STATUS_WORTIME0     0x37 /**<@brief Low byte of WOR timer*/
#define CCX_REG_STATUS_PKTSTATUS    0x38 /**<@brief Current GDOx status and packet status*/
#define CCX_REG_STATUS_VCO_VC_DAC   0x39 /**<@brief Current setting from PLL cal module*/
#define CCX_REG_STATUS_TXBYTES      0x3A /**<@brief Underflow and # of bytes in TXFIFO*/
#define CCX_REG_STATUS_RXBYTES      0x3B /**<@brief Overflow and # of bytes in RXFIFO*/

/******************************************************************************/
/* Status Register: MARCSTATE */
/******************************************************************************/

/**
 * @brief Values of the possible Main Radio Control FSM States. The field is
 * present within @ref CCX_REG_STATUS_MARCSTATE and can be retrieved with 
 * @ref CCX_MARCSTATE_GET_STATE.
 */
typedef enum {
    CCX_MARCSTATE_SLEEP              = 0x00, /**< SLEEP             */
    CCX_MARCSTATE_IDLE               = 0x01, /**< IDLE              */
    CCX_MARCSTATE_XOFF               = 0x02, /**< XOFF              */
    CCX_MARCSTATE_VCOON_MC           = 0x03, /**< MANCAL            */
    CCX_MARCSTATE_REGON_MC           = 0x04, /**< MANCAL            */
    CCX_MARCSTATE_MANCAL             = 0x05, /**< MANCAL            */
    CCX_MARCSTATE_VCOON              = 0x06, /**< FS_WAKEUP         */
    CCX_MARCSTATE_REGON              = 0x07, /**< FS_WAKEUP         */
    CCX_MARCSTATE_STARTCAL           = 0x08, /**< CALIBRATE         */
    CCX_MARCSTATE_BWBOOST            = 0x09, /**< SETTLING          */
    CCX_MARCSTATE_FS_LOCK            = 0x0A, /**< SETTLING          */
    CCX_MARCSTATE_IFADCON            = 0x0B, /**< SETTLING          */
    CCX_MARCSTATE_ENDCAL             = 0x0C, /**< CALIBRATE         */
    CCX_MARCSTATE_RX                 = 0x0D, /**< RX                */
    CCX_MARCSTATE_RX_END             = 0x0E, /**< RX                */
    CCX_MARCSTATE_RX_RST             = 0x0F, /**< RX                */
    CCX_MARCSTATE_TXRX_SWITCH        = 0x10, /**< TXRX_SETTLING     */
    CCX_MARCSTATE_RXFIFO_OVERFLOW    = 0x11, /**< RXFIFO_OVERFLOW   */
    CCX_MARCSTATE_FSTXON             = 0x12, /**< FSTXON            */
    CCX_MARCSTATE_TX                 = 0x13, /**< TX                */
    CCX_MARCSTATE_TX_END             = 0x14, /**< TX                */
    CCX_MARCSTATE_RXTX_SWITCH        = 0x15, /**< RXTX_SETTLING     */
    CCX_MARCSTATE_TXFIFO_UNDERFLOW   = 0x16  /**< TXFIFO_UNDERFLOW  */
} CcxMarcState;

/**
 * @brief Retrives the value of the Main Radio Control FSM state.
 *
 * @param _MS
 * Marcstate register value, obtained from @ref CCX_REG_STATUS_MARCSTATE.
 *
 * @return 
 * Value from @ref CcxMarcState
 */
#define CCX_MARCSTATE_GET_STATE(_MS)     CCX_GET_BITS_VALUE((_MS), 5, 0)


/**
 * @brief Determines if the Main Radio Control FSM state is transmit mode
 *
 * @param _MS
 * Marcstate register value, obtained from @ref CCX_REG_STATUS_MARCSTATE.
 *
 * @return
 * Logical true if MARC State suggests radio is in transmit mode.
 */
#define CCX_MARCSTATE_IS_IN_TX(_MS)                            \
    (CCX_MARCSTATE_GET_STATE((_MS)) == CCX_MARCSTATE_FSTXON || \
    CCX_MARCSTATE_GET_STATE((_MS)) == CCX_MARCSTATE_TX      || \
    CCX_MARCSTATE_GET_STATE((_MS)) == CCX_MARCSTATE_TX_END)

/******************************************************************************/
/* Status Register: PKTSTATUS */
/******************************************************************************/
/**
 * @brief Obtain CRC_OK from contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit CRC comparison matched (CRC_OK) value.
 */
#define CCX_PKTSTATUS_GET_CRC_OK(_PS)      CCX_GET_BITS_VALUE((_PS), 1, 7)

/* @brief Obtain CS from contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit Carrier Sense (CS) value.
 */
#define CCX_PKTSTATUS_GET_PQT_REACHED(_PS) CCX_GET_BITS_VALUE((_PS), 1, 5) 

/**
 * @brief Obtain PQT_REACHED from contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit Preamble Quality Reached (PQT_REACHED) value.
 */
#define CCX_PKTSTATUS_GET_PQT_REACHED(_PS) CCX_GET_BITS_VALUE((_PS), 1, 5) 

/**
 * @brief Obtain CCA from contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit Channel is Clear (CCA) value.
 */
#define CCX_PKTSTATUS_GET_CCA(_PS)         CCX_GET_BITS_VALUE((_PS), 1, 4)

/**
 * @brief Obtain SDF from the contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit Start of Frame Delimiter (SFD) bit.
 */
#define CCX_PKTSTATUS_GET_SFD(_PS)         CCX_GET_BITS_VALUE((_PS), 1, 3)

/**
 * @brief Obtain GDO2 from the contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit GD02 value.
 */
#define CCX_PKTSTATUS_GET_GDO2(_PS)        CCX_GET_BITS_VALUE((_PS), 1, 2)

/**
 * @brief Obtain GDB0 from the contents of the Packet Status register.
 * @param _PS Value obtained from reading @ref CCX_REG_STATUS_PKTSTATUS.
 * @return Single bit GD00 value.
 */
#define CCX_PKTSTATUS_GET_GDO0(_PS)        CCX_GET_BITS_VALUE((_PS), 1, 0)

/******************************************************************************/
/* Status Register: TXBYTES */
/******************************************************************************/
/**
 * @brief The size, in bytes, of the Transmit FIFO.
 */
#define CCX_TX_FIFO_MAX                  64

/**
 * @brief Obtain TXFIFO_UNDERFLOW from a prior reading of the TX Underflow and
 * Number of bytes register.
 * @param _TX Value obtained from reading @ref CCX_REG_STATUS_TXBYTES.
 * @return Single bit value of TXFIFO_UNDERFLOW.
 */
#define CCX_TX_BYTES_IS_UNDERFLOW(_TX)   CCX_GET_BITS_VALUE(_TX, 1, 7)

/**
 * @brief Obtain NUM_TXBYTES from a prior reading of TX Underflow and Number of
 * bytes register.
 * @param _TX Value obtained from reading @ref CCX_REG_STATUS_TXBYTES.
 * @return Number of bytes in TX FIFO (NUM_TXBYTES).
 */
#define CCX_TX_BYTES_IN_FIFO(_TX)        CCX_GET_BITS_VALUE(_TX, 7, 0)

/******************************************************************************/
/* Status Register: RXBYTES */
/******************************************************************************/
/**
 * @brief The size, in bytes, of the Receive FIFO.
 */
#define CCX_RX_FIFO_MAX                  64

/**
 * @brief Obtain RXFIFO_OVERFLOW from a prior reading of Rx Overflow and Number
 * of bytes register.
 * @param _RX Value obtained from reading @ref CCX_REG_STATUS_RXBYTES.
 * @return Single bit value of RXFIFO_OVERFLOW.
 */
#define CCX_RX_BYTES_IS_OVERFLOW(_RX)    CCX_GET_BITS_VALUE(_RX, 1, 7)

/**
 * @brief Obtain NUM_RXBYTES from a prior reading of RX Underflow and Number of
 * bytes register.
 * @param _RX Value obtained from reading @ref CCX_REG_STATUS_RXBYTES.
 * @return Number of bytes in RX FIFO (NUM_RXBYTES).
 */
#define CCX_RX_BYTES_IN_FIFO(_RX)        CCX_GET_BITS_VALUE(_RX, 7, 0)




/**@}*/

/******************************************************************************/
/**
 * @defgroup groupMultibyteRegisters Registers: Multibyte
 * @brief Multibyte Register Addresses
 *
 * These registers contain multiple bytes. A multiple byte burst to one of these
 * addresses will "top-up" the register's value rather than proceed to the next
 * register address.
 */
/**@{*/
/******************************************************************************/
#define CCX_REG_MULTIBYTE_PATABLE   0x3E /**<@brief PA Power Control Settings */
#define CCX_REG_MULTIBYTE_TXFIFO    0x3F /**<@brief Transmit FIFO (write access) */
#define CCX_REG_MULTIBYTE_RXFIFO    0x3F /**<@brief Receive FIFO (read addcess) */

/**@}*/

/******************************************************************************/
/**
 * @defgroup groupRegisterUtils Register Utilities
 * @brief Helper macros to extract bits from registers.
 *
 */
/**@{*/
/******************************************************************************/


/**
 * @brief Create a bit nask.
 * @param _WIDTH The width of the bitmask.
 * @param _OFFSET The offset of the lowest bit of the mask. 0 or greater.
 */
#define CCX_MASK(_WIDTH, _OFFSET)                        \
            (((2<<(_WIDTH - 1)) - 1) << (_OFFSET))

/**
 * @brief Retrive a bit mask from a variable. The value will be shifted so that
 * it occupies the least significant bytes.
 *
 * @param _BITFIELD The value containing a bitmask/bitfield to extract.
 * @param _WIDTH The width of the bitmask.
 * @param _OFFSET The offset of the lowest bit of the mask. 0 or greater. Once
 *                extracted the bitmask is shifted by this value to occupy the
 *                lowest bits.
 */
#define CCX_GET_BITS_VALUE(_BITFIELD, _WIDTH, _OFFSET)   \
            (((_BITFIELD) & (CCX_MASK(_WIDTH, _OFFSET))) >> _OFFSET)


/******************************************************************************/
/* Header Byte */
/******************************************************************************/

/**
 * @brief Mask of the Read bit within the header byte.
 */
#define CCX_HEADER_READ      CCX_MASK(1,7)

/**
 * @brief Mask of the Burst bit within the header byte.
 */
#define CCX_HEADER_BURST     CCX_MASK(1,6)

/**
 * @brief Mask of the Address bits within the header byte.
 */
#define CCX_HEADER_ADDRESS   CCX_MASK(6,0)

/******************************************************************************/
/* Chip Status Byte */
/******************************************************************************/

/**
 * @brief Possible states reported in the Chip Status byte. See @ref
 * CCX_SB_GET_STATE.
 */
typedef enum {
    CCX_STATE_IDLE               = 0X0, /**< IDLE state               */
    CCX_STATE_RX                 = 0X1, /**< Receive mode             */
    CCX_STATE_TX                 = 0X2, /**< Transmit mode            */
    CCX_STATE_FSTXON             = 0X3, /**< Fast TX ready            */
    CCX_STATE_CALIBRATE          = 0X4, /**< Frequency synthesizer calibration
                                             is running */ 
    CCX_STATE_SETTLING           = 0X5, /**< PLL is settling          */
    CCX_STATE_RXFIFO_OVERFLOW    = 0X6, /**< RX FIFO has overflowed   */
    CCX_STATE_TXFIFO_UNDERFLOW   = 0X7  /**< TX FIFO has overflowed   */
} Cc1101ChipStatusState;

/* CC1101 Chip Status Byte Bit Masks */

/**
 * @brief Retrieves the current main state machine mode from the Chip Status
 * byte.
 */
#define CCX_SB_GET_STATE(_SB)                CCX_GET_BITS_VALUE(_SB, 3, 4)

/**
 * @brief Retrieves the current bytes available in the RX FIFO / TX FIFO
 * depending on if the Chip Status byte was retrieved during a Read/Write
 * operation.
 */
#define CCX_SB_GET_FIFO_BYTES_AVAILABLE(_SB) CCX_GET_BITS_VALUE(_SB, 4, 0)

/******************************************************************************/
/* CC1101 Appended Bytes */
/******************************************************************************/

/** 
 * @brief Retrieves the RSSI value which is present within the first optional
 * status byte, appended to received data.
 *
 * @param _BYTE0 The first appended status byte.
 *
 * @return RSSI value.
 */
#define CCX_APD_B0_GET_RSSI(_BYTE0)  (_BYTE0)                  

/** 
 * @brief Retrieves CRC check value which is present within the second optional
 * status byte, appended to received data.
 *
 * @param _BYTE1 The second appended status byte.
 *
 * @return Single bit value of CRC_OK. If CRC for received data is OK, or CRC
 * check was disabled this will be `1`
 */
#define CCX_APD_B1_IS_CRC_OK(_BYTE1) ((_BYTE1) & CCX_MASK(1,7))

/** 
 * @brief Retrieves Link Quality Indication value, which is present within the
 * second optional status byte, appended to received data.
 *
 * @param _BYTE1 The second appended status byte.
 *
 * @return LQI value
 */
#define CCX_APD_B1_GET_LQI(_BYTE1)   (CCX_GET_BITS_VALUE((_BYTE1),7,0))


/**@}*/

#endif
