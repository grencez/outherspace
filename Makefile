

CFLAGS = -g3
#CFLAGS = -g -O2

CFLAGS = -O3
CFLAGS += -fopenmp
#CFLAGS += -DNDEBUG

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

all: hello main gui
	# Done!

OpenCLPath = /home/grencez/ati-stream-sdk-v2.3-lnx64
OpenCLLibPath = $(OpenCLPath)/lib/x86_64
hello: hello.c kdtree.c raytrace.c scene.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) -I $(OpenCLPath)/include $< -o $@ -L $(OpenCLLibPath) -lOpenCL

.PHONY: test-hello
test-hello: hello
	LD_LIBRARY_PATH=$(OpenCLLibPath) ./$<

main: cli.c kdtree.c main.c raytrace.c scene.c slist.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS)

gui: gui.c kdtree.c main.c raytrace.c scene.c slist.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) `pkg-config --cflags gtk+-2.0` $< -o $@ \
		`pkg-config --libs gtk+-2.0`

.PHONY: test
test: main
	#valgrind --track-origins=yes ./main
	./main

.PHONY: clean
clean:
	rm -f hello main gui

