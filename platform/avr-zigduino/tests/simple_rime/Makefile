CONTIKI_PROJECT = simple_rime
all: $(CONTIKI_PROJECT)
TARGET = avr-zigduino
CONTIKI = ../../../..
CFLAGS += -DNODE=$(NODE)
#UIP_CONF_IPV6 = 1
CFLAGS += -Wno-deprecated-declarations -D__PROG_TYPES_COMPAT__

include $(CONTIKI)/Makefile.include
