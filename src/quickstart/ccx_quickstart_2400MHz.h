/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//------------------------------------------------------------------------------
//  Description:  This file contains functions that configure the CC1100/2500
//  device.
//
//  Demo Application for MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.41
//------------------------------------------------------------------------------
/*
 * Alan Barr, 2017
 * Register settings where obtains from Texas Instruments slaa325a example code.
 * File was modified to define desired register values rather than set them.
 */

#define CCX_QUICKSTART_PATABLE_0_DMN        0xFB

// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 0.000000
// Return state:  Return to RX state upon leaving either TX or RX
// Datarate = 250.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2433.000000 MHz
// Channel spacing = 199.950000 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock

#define CCX_QUICKSTART_INITIAL_IOCFG2       0x0B // GDO2 output pin config.
#define CCX_QUICKSTART_INITIAL_IOCFG0       0x06 // GDO0 output pin config.
#define CCX_QUICKSTART_INITIAL_PKTLEN       0xFF // Packet length.
#define CCX_QUICKSTART_INITIAL_PKTCTRL1     0x05 // Packet automation control.
#define CCX_QUICKSTART_INITIAL_PKTCTRL0     0x05 // Packet automation control.
#define CCX_QUICKSTART_INITIAL_ADDR         0x01 // Device address.
#define CCX_QUICKSTART_INITIAL_CHANNR       0x00 // Channel number.
#define CCX_QUICKSTART_INITIAL_FSCTRL1      0x07 // Freq synthesizer control.
#define CCX_QUICKSTART_INITIAL_FSCTRL0      0x00 // Freq synthesizer control.
#define CCX_QUICKSTART_INITIAL_FREQ2        0x5D // Freq control word, high byte
#define CCX_QUICKSTART_INITIAL_FREQ1        0x93 // Freq control word, mid byte.
#define CCX_QUICKSTART_INITIAL_FREQ0        0xB1 // Freq control word, low byte.
#define CCX_QUICKSTART_INITIAL_MDMCFG4      0x2D // Modem configuration.
#define CCX_QUICKSTART_INITIAL_MDMCFG3      0x3B // Modem configuration.
#define CCX_QUICKSTART_INITIAL_MDMCFG2      0x73 // Modem configuration.
#define CCX_QUICKSTART_INITIAL_MDMCFG1      0x22 // Modem configuration.
#define CCX_QUICKSTART_INITIAL_MDMCFG0      0xF8 // Modem configuration.
#define CCX_QUICKSTART_INITIAL_DEVIATN      0x00 // Modem dev (when FSK mod en)
#define CCX_QUICKSTART_INITIAL_MCSM1        0x3F // MainRadio Cntrl State Machine
#define CCX_QUICKSTART_INITIAL_MCSM0        0x18 // MainRadio Cntrl State Machine
#define CCX_QUICKSTART_INITIAL_FOCCFG       0x1D // Freq Offset Compens. Config
#define CCX_QUICKSTART_INITIAL_BSCFG        0x1C // Bit synchronization config.
#define CCX_QUICKSTART_INITIAL_AGCCTRL2     0xC7 // AGC control.
#define CCX_QUICKSTART_INITIAL_AGCCTRL1     0x00 // AGC control.
#define CCX_QUICKSTART_INITIAL_AGCCTRL0     0xB2 // AGC control.
#define CCX_QUICKSTART_INITIAL_FREND1       0xB6 // Front end RX configuration.
#define CCX_QUICKSTART_INITIAL_FREND0       0x10 // Front end RX configuration.
#define CCX_QUICKSTART_INITIAL_FSCAL3       0xEA // Frequency synthesizer cal.
#define CCX_QUICKSTART_INITIAL_FSCAL2       0x0A // Frequency synthesizer cal.
#define CCX_QUICKSTART_INITIAL_FSCAL1       0x00 // Frequency synthesizer cal.
#define CCX_QUICKSTART_INITIAL_FSCAL0       0x11 // Frequency synthesizer cal.
#define CCX_QUICKSTART_INITIAL_FSTEST       0x59 // Frequency synthesizer cal.
#define CCX_QUICKSTART_INITIAL_TEST2        0x88 // Various test settings.
#define CCX_QUICKSTART_INITIAL_TEST1        0x31 // Various test settings.
#define CCX_QUICKSTART_INITIAL_TEST0        0x0B // Various test settings.

