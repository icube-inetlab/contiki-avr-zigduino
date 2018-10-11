CONTIKI = ../../../..
TARGET = avr-zigduino
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

all: example-collect example-neighbors example-broadcast \
	example-unicast example-runicast

CONTIKI_WITH_RIME = 1
include $(CONTIKI)/Makefile.include
