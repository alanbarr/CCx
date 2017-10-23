# MSP-EXP430G2 Transceiver Example

This example, targeted at the [MSP-EXP430G2](http://www.ti.com/tool/MSP-EXP430G2)
launchpad, in particular the MSP430G2553 MCU.

This example demonstrates basic RX and TX functionality.

The pin mapping is compatible with the [Anaren AIR Boosterpack/ 430BOOST-CC110L](
http://www.ti.com/tool/430BOOST-CC110L).

## Implementation Notes

- Written targeting TI's 
  [MSP430-GCC](http://www.ti.com/tool/msp430-gcc-opensource) compiler.

- By default, the device is in receive mode. 

- Pressing the switch will begin periodic transmission of messages at 1 second
  intervals. Pressing the switch a second time cancels the transmission.

- `Timer_A 0` is used to provide the periodic interrupts required to send a message
  once a second.

- `USCI_A0` is used to provide debug over backchannel UART (9600 baud).

- A persistent RED LED signifies a critical or unexpected error occurred.
  Details on the error should be available over UART.

- Green LED pin is shared with CC110L Chip Select.

- The pin mapping is available in `src/spi/ports/msp_exp430g2/msp_exp430g2.h`

- Ensure the Makefile variable `TOOL_PATH` is correct for your installation.
