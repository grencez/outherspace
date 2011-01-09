

CFLAGS = -g3
#CFLAGS = -O3

CFLAGS += -ansi -pedantic
CFLAGS += -Wall -Wextra

#CC = clang

#CC = gcc
#CC = g++
#CFLAGS += -DCOMPILER_HAS_BOOL

main: main.o
	$(CC) $(CFLAGS) $^ -o $@

test: main
	./main

