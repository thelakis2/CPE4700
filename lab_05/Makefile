CC = gcc
CPPFLAGS += $(shell pkg-config --cflags gtk4)
CFLAGS = -std=c11 -Wall -Wextra -O2 -MMD -MP
LDLIBS = -lgpiod -pthread -lm $(shell pkg-config --libs gtk4)
SOURCES = robotest.c balance_loop.c gui.c imu.c bno055.c encoder.c pwm.c
OBJECTS = $(SOURCES:.c=.o)
.PHONY: all clean
all: robotest
robotest: $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)
clean:
	rm -f $(OBJECTS) $(OBJECTS:.o=.d) robotest
-include $(OBJECTS:.o=.d)
