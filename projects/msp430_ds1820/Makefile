PROJECT_NAME := msp430-18s20

# Specify a cross-compiler prefix if needed
CROSS_PREFIX := msp430-elf-


# Build all files inside the following directories list:
VPATH := \
  . \
  ../../libs/ADC10 \

C_SRC_FILESLIST := ../../libs/UART/uart.c ../../libs/ringbuf.c

MCU := msp430g2452

# List of paths of extra directories to include
INCLUDE_PATHS_LIST := /opt/ti/gcc/include  ../../libs/UART

# Extra CFLAGS to use when compiling
USER_CFLAGS := -mmcu=$(MCU) -DUART_BAUDRATE=4800

# Extra LDFLAGS to use when linking
USER_LFLAGS := -mmcu=$(MCU) -L /opt/ti/gcc/include -T $(MCU).ld

DEBUG_LEVEL := Release

-include ~/shared/playground/Makefiles/Makefile_w_deps.mk

