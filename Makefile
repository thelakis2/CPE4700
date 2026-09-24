CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -O2
CPPFLAGS ?= -I./modules

MODULES = modules/pwm.c modules/encoder.c modules/speed_control.c

.PHONY: all clean

all: lab_04/lab04

lab_04/lab04: lab_04/main.c $(MODULES)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ lab_04/main.c $(MODULES) -lm

clean:
	rm -f lab_04/lab04 lab_04/*.csv