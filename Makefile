# Compiler
CC = gcc

# Output binary name
TARGET = controlSystem

# Source files
SRCS = controlSystem.c

# Libraries
LIBS = -lxlsxwriter -lpthread

# Compilation flags
CFLAGS = -Wall -Wextra -O2

# Default rule
all: $(TARGET)

# Build target
$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS) $(LIBS)

# Clean rule
clean:
	rm -f $(TARGET)

# Phony targets
.PHONY: all clean
