CCX_CSRC = $(CCX_SRC_DIR)/ccxxxx/ccx.c

CCX_INC = -I$(CCX_SRC_DIR)/../include\
          -I$(CCX_SRC_DIR)/spi \
          -I$(CCX_SRC_DIR)/cc1101


################################################################################
# CCX Compile in Tests 
################################################################################
ifeq ($(CCX_TESTS),1)
CCX_CSRC += $(CCX_SRC_DIR)/ccxxxx/ccx_tests.c
endif


################################################################################
# CCX Compile in Error logging
################################################################################
ifeq ($(CCX_LOG_ERRORS),1)
	CCX_DEF += -DCCX_LOG_ERRORS=1
else
	CCX_DEF += -DCCX_LOG_ERRORS=0
endif

################################################################################
# Quick Start Files
################################################################################
ifdef CCX_QS_FREQ
CCX_INC  += -I$(CCX_SRC_DIR)quickstart
CCX_CSRC += $(CCX_SRC_DIR)/quickstart/ccx_quickstart.c
endif
 
################################################################################
# Quick Start GD0 Pin
################################################################################
ifeq ($(CCX_QS_CONNECTED_GDO),0)
	CCX_DEF += -DCCX_QUICKSTART_GDO_CONNECTION=0
else ifeq ($(CCX_QS_CONNECTED_GDO),2)
	CCX_DEF += -DCCX_QUICKSTART_GDO_CONNECTION=2
else ifdef CCX_QS_FREQ
    $(error CCX_GDO is not a valid value)
endif

################################################################################
# Quick Start Radio Frequency
################################################################################
ifeq ($(CCX_QS_FREQ), 315)
	CCX_DEF += -DCCX_FREQ=315
else ifeq ($(CCX_QS_FREQ), 433)
	CCX_DEF += -DCCX_FREQ=433
else ifeq ($(CCX_QS_FREQ), 868)
	CCX_DEF += -DCCX_FREQ=868
else ifeq ($(CCX_QS_FREQ), 915)
	CCX_DEF += -DCCX_FREQ=915
else ifeq ($(CCX_QS_FREQ), 2400)
	CCX_DEF += -DCCX_FREQ=2400
else ifdef CCX_QS_FREQ
    $(error CCX_QS_FREQ is not a valid value)
endif
