CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)
CFLAGS += -DMAS=$(MASTER)
%.eep: %.o
        -$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
        --change-section-lma .eeprom=0 -O ihex $^ $@
TARGET = avr-zigduino
CONTIKI = ../../../..
CFLAGS += -Wno-deprecated-declarations -D__PROG_TYPES_COMPAT__

include $(CONTIKI)/Makefile.include
