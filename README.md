# CCxxxx

Note Doxygen generated HTML can be found [here](http://alanbarr.github.io/CCx).

This is a generic API initially targeting TI's CC110L/CC1101 line of RF
transceivers.  
It tries to wrap up some of the quirks in dealing with these
components, so they they don't have to be repeated for each MCU port.

As TI have made many of the registers common across the CC1100 and CC2500 range,
this driver should be compatible with other parts. Please notes that it is
possible CC110L/CC1101 specific features are used in this driver and the
examples.

# Modules 

This driver has three core modules:

* **CCxxxx**
The abstraction layer which should provide a platform independant
interface to the CC1101/CCxxxx.

* **QuickStart**
Convenience functions which depend on the
CCxxxx abstraction layer. They demonstrate one particular configuration of the
device, using the register configurations from TI's slaa325a example code.

* **SPI** 
The hardware specific part of this driver and will need to be tailored for the
Microcontroller being used.
The functionality required to be implemented can be found in
`src/spi/ccx_spi.h`.
Some example implementations of this can be found in `src/spi/ports`.


# Usage

## CCX

The file `src/ccx.mk` tries to quickly bring in the relevant files to your build
system. It should be sourced in your project Makefile and will provide the
following variables:

* `CCX_CSRC` - C source files
* `CCX_INC`  - include directories 
* `CCX_DEF`  - List of defines for the C preprocessor

An example of using this is available at
`examples/msp_exp430g2_transceiver/Makefile`. 

In order to use the CCX API you should:

* Source `src/ccx.mk` in your project Makefile
* Add `CCX_CSRC` to your C source/`CSRC`
* Add `CCX_INC` to your includes/`INC`
* Add `CCX_DEF` to your compiler flags/`CFLAGS`
* Include `ccx_api.h` in your C files

## Quickstart

To bring in Quickstart you should follow the steps above to use the CCX module.
Additionally, before sourcing `src/ccx.mk` in your
project Makefile you should:

* Set `CCX_QS_FREQ` to be either `315`, `433`, `868`, `915`, `2400`
* Set `CCX_QS_CONNECTED_GDO` to be either `0` or `2` to set the receive
interrupt as either `GD0` or `GD2`

Ensure that you include `ccx_quickstart.h` in your relevant C files.

## Debug

The following variables can be set in your Makefile before sourcing `src/ccx.mk`
to provide some convenient debugging.

* Set CCX_LOG_ERRORS to `1` to enable a user defined logging function 
(@ref ccxErrorLog) to be called when an error is encountered.
* Set `CCX_TESTS` to `1` to compile in a small group of tests.  
These can be called with @ref ccxRunTests.

# CC1101 Resources
* [CC1101 Datasheet](http://www.ti.com/lit/gpn/cc1101)
* [SPI Design Note DN503](http://www.ti.com/lit/an/swra112b/swra112b.pdf)
* [MSP430 Interface to CC1100/2500 Code LibraryTI (slaa235a)](http://www.ti.com/general/docs/litabsmultiplefilelist.tsp?literatureNumber=slaa325a)
