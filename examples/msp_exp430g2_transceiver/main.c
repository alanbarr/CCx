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
#include <string.h>
#include "ccx_api.h"
#include "ccx_quickstart.h"
#include "uart.h"
#include "msp_exp430g2.h"

#define UART_LINE_ENDING        "\r\n"

#define SWITCH_PRESSED          0x00

/* Timer A configured to interrupt every 25 ms. 40 gives 1 second between
 * transmissions. */
#define TIMERA_INTERRUPTS_TO_TX 40

static volatile struct {
    bool ccxMessagesToReceive;
    bool switchPressed;
    bool timerExpired;
} events;

static struct {
    bool isTransmitting;
} state;

static volatile struct {
    uint16_t switchInterruptCount;
    uint16_t rxInterruptCount;
    uint16_t timerAInterruptCount;
} debug;


/* 0xAB\0 */
void printHexByte(uint8_t i)
{
    uint8_t nibble = 0;
    char str[5];

    str[0] = '0';
    str[1] = 'x';

    nibble = i>>4;

    if (nibble < 0xA)
    {
        nibble += '0';
    }
    else 
    {
        nibble += 'A' - 0xA;
    }

    str[2] = nibble;

    nibble = i & 0xF;

    if (nibble < 0xA)
    {
        nibble += '0';
    }
    else 
    {
        nibble += 'A' - 0xA;
    }

    str[3] = nibble;

    str[4] = '\0';

    uartPrint(str);
}

static void unexpectedError(uint8_t error)
{
    uartPrint("ERROR: ");
    printHexByte(error);
    uartPrint(UART_LINE_ENDING);

    while(1)
    {
        P1OUT |= PIN_RED_LED_PORT1;
    }
}

void ccxErrorLog(uint8_t status, char *file, char *line)
{
    uartPrint("Error Occured\n\r");
    uartPrint("Status: ");
    printHexByte(status);
    uartPrint(" File: ");
    uartPrint(file);
    uartPrint(" Line: ");
    uartPrint(line);
    uartPrint(UART_LINE_ENDING);
}

static uint8_t debugChipInformation(void)
{
    uint8_t status = CCX_ERROR_INTERNAL;
    uint8_t read = 0xAB;

    /**************************************************************************/
    if ((status = ccxReadStatusRegister(CCX_REG_STATUS_PARTNUM, &read)))
    {
        unexpectedError(status);
    }

    uartPrint("Partnum: ");
    printHexByte(read);
    uartPrint(UART_LINE_ENDING);

    /**************************************************************************/
    if ((status = ccxReadStatusRegister(CCX_REG_STATUS_VERSION, &read)))
    {
        unexpectedError(status);
    }

    uartPrint("Version: ");
    printHexByte(read);
    uartPrint(UART_LINE_ENDING);

    /**************************************************************************/
    if ((status = ccxReadByte(CCX_REG_IOCFG2, &read)))
    {
        unexpectedError(status);
    }

    uartPrint("IOCFG2: ");
    printHexByte(read);
    uartPrint(UART_LINE_ENDING);

    /**************************************************************************/
    if ((status = ccxReadByte(CCX_REG_IOCFG1, &read)))
    {
        unexpectedError(status);
    }

    uartPrint("IOCFG1: ");
    printHexByte(read);
    uartPrint(UART_LINE_ENDING);

    /**************************************************************************/
    if ((status = ccxReadByte(CCX_REG_IOCFG0, &read)))
    {
        unexpectedError(status);
    }

    uartPrint("IOCFG0: ");
    printHexByte(read);
    uartPrint(UART_LINE_ENDING);

    return status;
}

void switchDebounce(void)
{
    uint8_t sequentialReleasedCtr = 0;

    while ((P1IN & PIN_SWITCH_PORT1) == SWITCH_PRESSED);

    while (sequentialReleasedCtr != 30)
    {
        if (!((P1IN & PIN_SWITCH_PORT1) == SWITCH_PRESSED))
        {
            sequentialReleasedCtr++;
        }
        else
        {
            sequentialReleasedCtr = 0;
        }

        /* Sleep for 1 ms */
        __delay_cycles(16000);
    }   
}

void processEvents(void)
{
    static uint8_t rxBuffer[26];
    uint8_t lqi = 0;
    uint8_t rssi = 0;
    uint8_t status = CCX_OK;
    const char * const txMsg = "abcdefghijklmnopqrstuvwxyz";

    if (events.switchPressed)
    {
        switchDebounce();

        if (state.isTransmitting)
        {
            /* Timer A.0 Configuration
             * Reset the timer
             * Stop interrupts
             */
            TA0CTL  |= TACLR;
            TA0CTL &= ~TAIE;

            state.isTransmitting = false;
        }
        else
        {
            /* Timer A.0 Initialisation Configuration
             * SMCLK / 8 = 16 MHz / 8 = 2 MHz
             * Up Mode
             * Count to 50,000 (40 interrupts per second, 25 ms granularity).
             * Enable Interrupt
             */
            TA0CTL  = TASSEL_2 | ID_3 | MC_1;
            TA0CCR0 = 50000 - 1;
            TA0CTL |= TAIE;

            state.isTransmitting = true;
        }

        events.switchPressed = false;
    }

    if (events.ccxMessagesToReceive)
    {
        uint8_t bufOrPktLength = sizeof(rxBuffer);

        uartPrint("Receiving message..." UART_LINE_ENDING);

        if ((status = ccxQuickstartReceiveVariableLength(rxBuffer,
                                                        &bufOrPktLength,
                                                        &rssi,
                                                        &lqi)))
        {

            /* CRC error isn't critcial, just flush the FIFO and continue on */
            if (status == CCX_ERROR_RX_CRC)
            {
                uartPrint("... Bad CRC!" UART_LINE_ENDING);
                if ((status = ccxStrobe(CCX_REG_STROBE_SIDLE)))
                {
                    unexpectedError(status);
                }

                if ((status = ccxStrobe(CCX_REG_STROBE_SFRX)))
                {
                    unexpectedError(status);
                }

                if ((status = ccxStrobe(CCX_REG_STROBE_SRX)))
                {
                    unexpectedError(status);
                }

                events.ccxMessagesToReceive = false;
                return;
            }
            else
            {
                unexpectedError(status);
            }
        }

        uartPrint("... received" UART_LINE_ENDING);

        if (bufOrPktLength != strlen(txMsg))
        {
            unexpectedError(CCX_ERROR_ASSERT);
        }

        if (memcmp(txMsg, rxBuffer, strlen(txMsg)) != 0)
        {
            unexpectedError(CCX_ERROR_ASSERT);
        }

        uartPrint("RSSI: ");
        printHexByte(rssi);
        uartPrint(UART_LINE_ENDING);

        uartPrint("LQI: ");
        printHexByte(lqi);
        uartPrint(UART_LINE_ENDING);

        events.ccxMessagesToReceive = false;
    }

    if (events.timerExpired)
    {
        uartPrint("Transmitting message..." UART_LINE_ENDING);

        if ((status = ccxQuickstartTransmitVariableLength(
                                                1,
                                                (const uint8_t *)txMsg, 
                                                strlen(txMsg))))
        {
            unexpectedError(status);
        }

        uartPrint("... transmitted." UART_LINE_ENDING);

        events.timerExpired = false;
    }

}

int main(void)
{
    uint8_t status = CCX_ERROR_INTERNAL;
    uint8_t rxBytesAvailable = 0;

    /* Stop watchdog */
    WDTCTL = WDTPW | WDTHOLD;

    /* Run at 16 MHz */
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL  = CALDCO_16MHZ; 

    /* LED LED is output, low */
    P1DIR |= PIN_RED_LED_PORT1;
    P1OUT &= ~PIN_RED_LED_PORT1;

    /* Switch configuration
     *  input
     *  internal resistor
     *  pullup
     *  clear interrupt flag
     *  interrupt enable
     *  low to high 
     */
    P1DIR &= ~PIN_SWITCH_PORT1;
    P1REN |=  PIN_SWITCH_PORT1;
    P1OUT |=  PIN_SWITCH_PORT1;
    P1IFG &= ~PIN_SWITCH_PORT1;
    P1IE  |=  PIN_SWITCH_PORT1;
    P1IES &= ~PIN_SWITCH_PORT1;

    uartInit();

    uartPrint("#############################################" UART_LINE_ENDING);
    uartPrint("starting..." UART_LINE_ENDING);

    if ((status = ccxRunTests()))
    {
        unexpectedError(status);
    }

    if ((status = ccxQuickstartInit()))
    {
        unexpectedError(status);
    }

    if ((status = debugChipInformation()))
    {
        unexpectedError(status);
    }

    CCX_STATUS_CHECK(ccxStrobe(CCX_REG_STROBE_SRX), status);

    while (1)
    {
        CCX_STATUS_CHECK(ccxReadStatusRegister(CCX_REG_STATUS_RXBYTES, 
                                               &rxBytesAvailable),
                         status);

        if (!CCX_RX_BYTES_IN_FIFO(rxBytesAvailable))
        {
            P2IFG &= ~PIN_GDO_PORT2;
            P2IE  |=  PIN_GDO_PORT2;

            P1IFG &= ~PIN_SWITCH_PORT1;
            P1IE  |=  PIN_SWITCH_PORT1;

            uartPrint("sleeping..." UART_LINE_ENDING);
            __bis_SR_register(LPM0_bits | GIE);
            __no_operation();
            uartPrint("...awake" UART_LINE_ENDING);
        }
        else 
        {
            events.ccxMessagesToReceive = true;
        }

        processEvents();
    }
}

void __attribute__ ((interrupt(PORT2_VECTOR))) interruptPort2Vector(void)
{
    debug.rxInterruptCount++;

    if (P2IFG & PIN_GDO_PORT2)
    {
        P2IE  &= ~PIN_GDO_PORT2;
        P2IFG &= ~PIN_GDO_PORT2;
        events.ccxMessagesToReceive = true;
    }

    __bic_SR_register_on_exit(LPM0_bits | GIE);
}


void __attribute__ ((interrupt(PORT1_VECTOR))) interruptPort1Vector(void)
{
    debug.switchInterruptCount++;

    if (P1IFG & PIN_SWITCH_PORT1)
    {
        P1IE  &= ~PIN_SWITCH_PORT1;
        P1IFG &= ~PIN_SWITCH_PORT1;
        events.switchPressed = true;
    }

    __bic_SR_register_on_exit(LPM0_bits | GIE);
}


void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) interruptTimer0A1Vector(void)
{
    static uint8_t overflowCount = 0;

    debug.timerAInterruptCount++;

    if (TA0IV != TA0IV_TAIFG)
    {
        P1OUT |= PIN_RED_LED_PORT1;
        while(1);
    }

    overflowCount++;

    if (overflowCount >= TIMERA_INTERRUPTS_TO_TX)
    {
        overflowCount = 0;
        events.timerExpired = true;
        __bic_SR_register_on_exit(LPM0_bits | GIE);
    }
}
