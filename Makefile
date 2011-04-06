
#CC = clang
#CC = gcc
#CC = g++

CONFIG = fast
CONFIG = fast openmp
#CONFIG = fast openmp noassert
#CONFIG = fast openmp benchmark
#CONFIG = fast mpi
#CONFIG = debug
#CONFIG = fast noassert
#CONFIG = benchmark snappy debug
#CONFIG = benchmark snappy debug openmp
#CONFIG = ultradebug

CONFIG += ansi
#CONFIG += c99


ifeq ($(CC),g++)
	CONFIG += c++
endif

ifneq ($(CC),clang)
	CFLAGS += -combine
endif
CFLAGS += -fwhole-program
CFLAGS += -Wall -Wextra


## Serious debugging is about to happen.
ifneq (,$(findstring ultradebug,$(CONFIG)))
	CONFIG = $(filter-out snappy fast debug,$(CFLAGS))
	CFLAGS += -g3
endif
## Go really fast.
ifneq (,$(findstring fast,$(CONFIG)))
	CFLAGS += -O3
	#CFLAGS += -ffast-math
	#CFLAGS += -march=native -mtune=native
endif
## Go pretty fast.
ifneq (,$(findstring snappy,$(CONFIG)))
	CFLAGS += -O2
endif
## Add debugging symbols.
ifneq (,$(findstring debug,$(CONFIG)))
	CFLAGS += -g
endif

## Enable benchmarking.
ifneq (,$(findstring benchmark,$(CONFIG)))
	CFLAGS += -DBENCHMARKING
endif
## Disable assertions.
ifneq (,$(findstring noassert,$(CONFIG)))
	CFLAGS += -DNDEBUG
endif
## Do we have bool type?
ifneq (,$(findstring c++,$(CONFIG)))
	CFLAGS += -DCOMPILER_HAS_BOOL
endif
## Use the C99 standard.
ifneq (,$(findstring c99,$(CONFIG)))
	CFLAGS += -std=c99
endif
## Stick to the ANSI standard.
ifneq (,$(findstring ansi,$(CONFIG)))
	CFLAGS += -ansi -pedantic
endif
## Allow parallelism.
ifneq (,$(findstring openmp,$(CONFIG)))
	CFLAGS += -fopenmp
endif
## Allow distributed parallelism.
ifneq (,$(findstring mpi,$(CONFIG)))
	CC = mpicc
	CFLAGS += -DDistribCompute
endif


CSources = kdtree.c raytrace.c scene.c simplex.c slist.c space.c util.c xfrm.c


LFLAGS += -lm -lz

all: cli gui hello
	# Done!

OpenCLPath = /home/grencez/ati-stream-sdk-v2.3-lnx64
OpenCLLibPath = $(OpenCLPath)/lib/x86_64
hello: hello.c $(CSources)
	$(CC) $(CFLAGS) -I $(OpenCLPath)/include $< -o $@ \
		-L $(OpenCLLibPath) -lOpenCL $(LFLAGS)

.PHONY: test-hello
test-hello: hello
	LD_LIBRARY_PATH=$(OpenCLLibPath) ./$<

cli: cli.c compute.c main.c $(CSources)
	$(CC) $(CFLAGS) $< -o $@ $(LFLAGS)

gui: gui.c compute.c motion.c $(CSources)
	$(CC) $(CFLAGS) `pkg-config --cflags gtk+-2.0` \
		`sdl-config --cflags` \
		$< -o $@ \
		`pkg-config --libs gtk+-2.0` \
		`sdl-config --libs`

.PHONY: test
test: cli
	#valgrind --track-origins=yes ./$<
	./$<

.PHONY: clean
clean:
	rm -f hello cli gui

