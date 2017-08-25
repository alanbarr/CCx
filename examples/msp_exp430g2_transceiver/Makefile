# TI's MSP430-GCC appears to have some issues finding header files and linker
# scripts.
# Need to ensure TOOL_PATH points to the toolchain's install directory.
TOOL_PATH=/mnt/ext4_part/msp430/ti/msp430_gcc/
TOOL_INC=$(TOOL_PATH)/include
TOOL_PREFIX=msp430-elf-
MCU=msp430g2553 

CC=$(TOOL_PREFIX)gcc
SIZE=$(TOOL_PREFIX)size
OD=$(TOOL_PREFIX)objdump

TOOL_FLAGS= -mmcu=$(MCU) -I$(TOOL_INC) -L$(TOOL_INC) 

OD_FLAGS=--disassemble-all --section-headers --source

CFLAGS+=-Wall -Wextra -Wpedantic -std=c99 
CFLAGS+=-ggdb -g3
CFLAGS+=-minrt
CFLAGS+=-O2

OUTDIR=output

CSRC=main.c uart.c
INC=-I.

################################################################################
CCX_SRC_DIR=../../src
CCX_QS_FREQ=868
CCX_QS_CONNECTED_GDO=0
CCX_TESTS=1
CCX_LOG_ERRORS=1

include $(CCX_SRC_DIR)/ccx.mk

CSRC+= $(CCX_CSRC)
INC+=$(CCX_INC) 
CFLAGS+=$(CCX_DEF)

CSRC+=$(CCX_SRC_DIR)/spi/ports/msp_exp430g2/msp_exp430g2.c 
INC+=-I$(CCX_SRC_DIR)/spi/ports/msp_exp430g2/
################################################################################

ELF_OUTPUT=$(OUTDIR)/msp430.elf
OD_OUTPUT=$(OUTDIR)/msp430.dump

all: $(ELF_OUTPUT) $(OD_OUTPUT)

$(ELF_OUTPUT): Makefile $(OUTDIR) $(CSRC)
	echo "CSRC is $(CSRC)"
	$(CC) $(TOOL_FLAGS) $(CFLAGS) $(CSRC) $(INC) -o $(ELF_OUTPUT)
	$(SIZE) $(ELF_OUTPUT)
	@sha1sum  $(ELF_OUTPUT)
	@readlink -f $(ELF_OUTPUT)

$(OD_OUTPUT): Makefile $(ELF_OUTPUT)
	$(OD) $(OD_FLAGS) $(ELF_OUTPUT) > $(OD_OUTPUT)

$(OUTDIR):
	mkdir -p $@

clean:
	rm $(OUTDIR)/*
	rmdir $(OUTDIR)



