

CFLAGS = -g3
CFLAGS = -O3

CFLAGS += -ansi -pedantic
CFLAGS += -Wall -Wextra
#CFLAGS += -funsafe-loop-optimizations
#CFLAGS += -fstrict-aliasing
#CFLAGS += -ftree-loop-linear
#CFLAGS += -funsafe-math-optimizations
#CFLAGS += -ftree-loop-im
#CFLAGS += -march=native
#CFLAGS += -ftree-loop-distribution
#CFLAGS += -ftree-loop-ivcanon
#CFLAGS += -fivopts

LFLAGS += -lm

#CC = clang

#CC = gcc
#CC = g++
#CFLAGS += -DCOMPILER_HAS_BOOL

#main: kdtree.o main.o util.o space.o
main: main.c
	$(CC) $(CFLAGS) $^ -o $@ $(LFLAGS)

gui: gui.c
	$(CC) $(CFLAGS) `pkg-config --cflags gtk+-2.0` $< -o $@ \
		`pkg-config --libs gtk+-2.0`

test: main
	#valgrind --track-origins=yes ./main
	./main

.PHONY: clean
clean:
	rm -f main gui kdtree.o main.o util.o space.o

