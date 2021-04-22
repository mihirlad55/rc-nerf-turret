CC := avr-gcc
LD := avr-ld
OBJCOPY := avr-objcopy

INCDIRS := /usr/avr/include include
INC_PARAMS=$(foreach d, $(INCDIRS), -I$d)

CFLAGS := -O2 -mmcu=atmega328p -Wall
CPPFLAGS := -I
LDFLAGS := -m avr4

SRC := src/turret.c
OBJ := turret.o

.PHONY: all clean

all: turret.hex

clean:
	rm -f ${OBJ} turret.hex

turret.o: src/turret.c
	${CC} ${CFLAGS} ${INC_PARAMS} -o $@ $<

turret.hex: turret.o
	${OBJCOPY} -O ihex $< $@

