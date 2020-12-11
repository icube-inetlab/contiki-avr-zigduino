CONTIKI = ../../../..
TARGET = avr-zigduino
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

all: collect-sensors

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include
CONTIKI_PLAT_DEFS = -DF_CPU=16000000UL -DAUTO_CRC_PADDING=2
