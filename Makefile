

CFLAGS = -g3
CFLAGS = -O3

CFLAGS += -ansi -pedantic
CFLAGS += -Wall -Wextra
LFLAGS += -lm

#CC = clang

#CC = gcc
#CC = g++
#CFLAGS += -DCOMPILER_HAS_BOOL

main: main.o
	$(CC) $(CFLAGS) $^ -o $@ $(LFLAGS)

gui: gui.c
	$(CC) $(CFLAGS) `pkg-config --cflags gtk+-2.0` $< -o $@ \
		`pkg-config --libs gtk+-2.0`

test: main
	#valgrind --track-origins=yes ./main
	./main

