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

/*******************************************************************************
 * MSP430 to CC110x
 *
 *  SPI Hardware: UCB0
 *
 *  Package Pin     MSP430 Pin          CC110x Purpose
 *   7              1.5                 Clock
 *  14              1.6                 Slave Out
 *  15              1.7                 Slave In
 *  18              2.7                 Chip Select
 *  19              2.6                 GD0
 *
 * Usage requirements:
 *  Port 2 Bit 6 (GDO) is set to be an interrupt.
 *  User application _must_ catch and clear this interrupt. 
 *  It should begin the process of receiving a message upon detecting the
 *  interrupt.
 ******************************************************************************/

#include <msp430.h>
#include "msp_exp430g2.h"
#include "ccx_spi.h"


uint8_t ccxSpiPinChipSelectSet(uint8_t setValue)
{
    if (setValue)
    {
        P2OUT |= PIN_SELECT_PORT2;
    }
    else
    {
        P2OUT &= ~PIN_SELECT_PORT2;
    }
    return CCX_OK;
}

uint8_t ccxSpiPinSlaveOutRead(uint8_t *currentValue)
{
    *currentValue = (P1IN & PIN_SLAVE_OUT_PORT1) ? 1 : 0;

    return CCX_OK;
}

uint8_t ccxSpiTransaction(uint8_t address,
                          const uint8_t *writeBytes,
                          uint8_t writeLength,
                          uint8_t *readBytes,
                          uint8_t readLength)
{
    uint8_t dataByteIndex = 0;

    /**************************************************************************/
    /* Send address byte */
    /**************************************************************************/
    while (!(IFG2 & UCB0TXIFG));    
    UCB0TXBUF = address;
    while (!(IFG2 & UCB0RXIFG));
    IFG2 &= ~UCB0RXIFG;

    if (readLength)
    {
        while (readLength)
        {
            while (!(IFG2 & UCB0TXIFG));    

            UCB0TXBUF = 0;

            while (!(IFG2 & UCB0RXIFG));

            readBytes[dataByteIndex] = UCB0RXBUF;
            readLength--;
            dataByteIndex++;
        }
    }
    else if (writeLength)
    {
        while (writeLength)
        {
            while (!(IFG2 & UCB0TXIFG));    

            UCB0TXBUF = writeBytes[dataByteIndex];             

            writeLength--;
            dataByteIndex++;

            while (!(IFG2 & UCB0RXIFG));
            IFG2 &= ~UCB0RXIFG;
        }
    }

    return CCX_OK;
}

uint8_t ccxSpiDelay25us(void)
{
    /**************************************************************************/
    /* At 16 MHz, 400 clock cycles is 25 us */
    /**************************************************************************/
    __delay_cycles(400);

    return CCX_OK;
}

uint8_t ccxSpiInit(void)
{
    /**************************************************************************/
    /* Place SPI peripherial into reset */
    /**************************************************************************/
    UCB0CTL1 |= UCSWRST;

    /**************************************************************************/
    /* 3 SPI Pins to SPI peripherial */
    /**************************************************************************/
    P1SEL  |= PIN_CLOCK_PORT1 | PIN_SLAVE_OUT_PORT1 | PIN_SLAVE_IN_PORT1;
    P1SEL2 |= PIN_CLOCK_PORT1 | PIN_SLAVE_OUT_PORT1 | PIN_SLAVE_IN_PORT1;

    /**************************************************************************/
    /* Chip select: don't use crystal, pin output, high */
    /**************************************************************************/
    P2SEL &= ~PIN_SELECT_PORT2;
    P2DIR |=  PIN_SELECT_PORT2;
    P2OUT |=  PIN_SELECT_PORT2;

    /**************************************************************************/
    /* GD0 pin:
     *  don't use crystal
     *  input
     *  clear flag
     *  enable interrupt
     *  low to high  */
    /**************************************************************************/
    P2SEL &= ~PIN_GDO_PORT2;
    P2DIR &= ~PIN_GDO_PORT2;
    P2IFG &= ~PIN_GDO_PORT2;
    P2IE  |=  PIN_GDO_PORT2;
    P2IES &= ~PIN_GDO_PORT2;
    
    /**************************************************************************/
    /* SPI Configuration:
     *  - Data captured on first edge
     *  - SPI Master
     *  - 3 Pin SPI 
     *  - Synchronous Mode
     */
    /**************************************************************************/
    UCB0CTL0 = UCCKPH | UCMST | UCMSB | UCSYNC;

    /**************************************************************************/
    /* SPI Clocking 
     * - BRCLK source is SMCLK
     * - Pre-scaler 2. Assuming 16 MHz SMCLK gives 8 MHz.
     *   Assuming 16 MHz, 62.5 ns per instruction, 100 ns delay to exceed 6.5
     *   MHz clock shouldn't be concern.  
     */
    /**************************************************************************/
    UCB0CTL1 |= UCSSEL_2;
    UCB0BR0   = 2;
    UCB0BR1   = 0;

    /**************************************************************************/
    /* Remove SPI peripheral from reset */
    /**************************************************************************/
    UCB0CTL1 &= ~UCSWRST;

    return CCX_OK;
}

uint8_t ccxSpiShutdown(void)
{
    /**************************************************************************/
    /* Place SPI peripherial into reset */
    /**************************************************************************/
    UCB0CTL1 |= UCSWRST;

    /**************************************************************************/
    /* Disconnect SPI pins from SPI peripheral */
    /**************************************************************************/
    P1SEL  &= ~(PIN_CLOCK_PORT1 | PIN_SLAVE_OUT_PORT1 | PIN_SLAVE_IN_PORT1);
    P1SEL2 &= ~(PIN_CLOCK_PORT1 | PIN_SLAVE_OUT_PORT1 | PIN_SLAVE_IN_PORT1);

    /**************************************************************************/
    /* Disable GDO pin interrupt */
    /**************************************************************************/
    P2IE |= (PIN_GDO_PORT2);

    /**************************************************************************/
    /* Chip select pin to ouput high, the input */
    /**************************************************************************/
    P2OUT |= PIN_SELECT_PORT2;
    P2DIR &= ~PIN_SELECT_PORT2;

    return CCX_OK;
}

