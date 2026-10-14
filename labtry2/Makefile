# Target executable name
TARGET = robotest

# Compiler and optimization flags
CC = gcc
CFLAGS = -Wall -Wextra -O3

# Standard hardware / system libraries
LIBS = -lgpiod -lpthread -lm

# Automatically find all .c files in the current directory
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

# Default rule to build the application
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# Compile .c files to .o files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -f *.o $(TARGET)

.PHONY: all clean
