# name of your application
APPLICATION = arduino_hello-world

# If no BOARD is found in the environment, use this default:
BOARD ?= radino32

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

USEMODULE += arduino

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include
