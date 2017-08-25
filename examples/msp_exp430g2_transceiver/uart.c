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

#include <msp430.h>
#include <string.h>
#include "uart.h"
#include "msp_exp430g2.h"

void uartInit(void)
{
    UCA0CTL1 |= UCSWRST;
    
    P1SEL  |= PIN_UARTTX_PORT1 | PIN_UARTRX_PORT1;
    P1SEL2 |= PIN_UARTTX_PORT1 | PIN_UARTRX_PORT1;

    /* Use SMCLK */
    UCA0CTL1 |= UCSSEL_2;
    
    UCA0BR0 = 0xFF &  (unsigned short)(16000000UL / UART_HW_BAUD);
    UCA0BR1 = 0xFF & ((unsigned short)(16000000UL / UART_HW_BAUD) >> 8);

    /* Modulation UCBRSx = 1 */
    UCA0MCTL = UCBRS0;                        

    UCA0CTL1 &= ~UCSWRST;               
}

void uartPrint(const char *string)
{
    while (string != NULL && *string != 0)
    {
        UCA0TXBUF = *string;
        string++;
        while (!(IFG2 & UCA0TXIFG));
    }
}

void uartDisable(void)
{
    UCA0CTL1 |= UCSWRST;
    P1SEL    &= ~(PIN_UARTTX_PORT1 | PIN_UARTRX_PORT1);
    P1SEL2   &= ~(PIN_UARTTX_PORT1 | PIN_UARTRX_PORT1);
}

