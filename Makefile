

CFLAGS = -g3
CFLAGS = -g -O2

#CFLAGS = -s -O2
#CFLAGS = -g -pg -O2

CFLAGS = -O3 -DNDEBUG
#CFLAGS += -fompenmp -combine -fwhole-program

CFLAGS += -ansi -pedantic
CFLAGS += -Wall -Wextra

#CFLAGS += -ffast-math
#CFLAGS += -march=native -mtune=native

LFLAGS += -lm

#CC = clang

#CC = gcc
#CC = g++
#CFLAGS += -DCOMPILER_HAS_BOOL

all: hello cli gui
	# Done!

OpenCLPath = /home/grencez/ati-stream-sdk-v2.3-lnx64
OpenCLLibPath = $(OpenCLPath)/lib/x86_64
hello: hello.c kdtree.c raytrace.c scene.c slist.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) -I $(OpenCLPath)/include $< -o $@ -L $(OpenCLLibPath) -lOpenCL

.PHONY: test-hello
test-hello: hello
	LD_LIBRARY_PATH=$(OpenCLLibPath) ./$<

cli: cli.c kdtree.c main.c raytrace.c scene.c slist.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS)

gui: gui.c kdtree.c main.c raytrace.c scene.c slist.c space.c util.c xfrm.c
	$(CC) $(CFLAGS) `pkg-config --cflags gtk+-2.0` $< -o $@ \
		`pkg-config --libs gtk+-2.0`

.PHONY: test
test: cli
	#valgrind --track-origins=yes ./cli
	./cli

.PHONY: clean
clean:
	rm -f hello cli gui

