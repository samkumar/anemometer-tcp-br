APPLICATION = openthread_anemometer_tcp_br

# If no BOARD is found in the environment, use this default:
BOARD ?= samr21-xpro

# These are the boards that OpenThread stack has been tested on
BOARD_WHITELIST := hamilton samr21-xpro iotlab-m3 fox iotlab-a8-m3

# This has to be the absolute path to the RIOT base directory:
RIOTBASE = ../RIOT-OS

# This is to set the proper programmer for board
PROGRAMMER ?= jlink

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP -Wall

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

# NCP: A Network Co-Processor is used with wpantund software. wpantund is a user-space network
# interface driver/daemon that provides a native IPv6 network interface to a low-power wireless Network Co-Processor
# FTD (see openthread_test_ftd): NCP needs Full Thread Device to act as a router in the network
USEPKG += openthread
USEMODULE += openthread-ftd
USEMODULE += openthread-ncp-ftd

#Define UART_BAUDRATE, PANID, CHANNEL, and TXPOWER used by default
OPENTHREAD_UART_BAUDRATE ?= 921600
CFLAGS += -DOPENTHREAD_UART_BAUDRATE=${OPENTHREAD_UART_BAUDRATE}
OPENTHREAD_PANID ?= 0xbeef
OPENTHREAD_CHANNEL ?= 26
OPENTHREAD_TXPOWER ?= -8
CFLAGS += -DOPENTHREAD_PANID=${OPENTHREAD_PANID}
CFLAGS += -DOPENTHREAD_CHANNEL=${OPENTHREAD_CHANNEL}
CFLAGS += -DOPENTHREAD_TXPOWER=${OPENTHREAD_TXPOWER}
HOSTNAME ?= 17
CFLAGS += -DOPENTHREAD_SOURCE=${HOSTNAME}

ifneq (,$(filter hamilton samr21-xpro,$(BOARD)))
  DRIVER := at86rf233
endif
ifneq (,$(filter iotlab-m3 fox iotlab-a8-m3,$(BOARD)))
  DRIVER := at86rf231
endif

ifneq (,$(filter at86rf2%,$(DRIVER)))
  FEATURES_REQUIRED += periph_spi periph_gpio
  CFLAGS += -DAUTO_CSMA_EN=1
endif

#CFLAGS += -DHIGH_DATA_RATE=1

USEMODULE += $(DRIVER)

USEMODULE += xtimer
USEMODULE += rethos
USEMODULE += random
USEMODULE += ps
USEMODULE += schedstatistics
CFLAGS += -DRADIO_DUTYCYCLE_MONITOR=1
CFLAGS += -DCPU_DUTYCYCLE_MONITOR=1

# rethos baudrate can be configured from make command
RETHOS_BAUDRATE ?= $(OPENTHREAD_UART_BAUDRATE)
CFLAGS += '-DRETHOS_UART=UART_DEV(0)' -DRETHOS_BAUDRATE=$(RETHOS_BAUDRATE)
FEATURES_REQUIRED += periph_uart

USEMODULE += rtt_stdio
CFLAGS    += -DRTT_STDIO_DISABLE_STDIN
CFLAGS    += -DTHREAD_STACKSIZE_DEFAULT=2048
CFLAGS += -DISR_STACKSIZE=1596

#required for C++ compiling
CXXEXFLAGS += -fno-rtti
USEMODULE += cpp11-compat

include $(RIOTBASE)/Makefile.include
