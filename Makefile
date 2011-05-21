
#CC = clang
#CC = gcc
#CC = llvm-gcc

DFLAGS += -DNDimensions=4

CONFIG = fast
CONFIG = fast openmp
#CONFIG = fast openmp benchmark
#CONFIG = fast openmp noassert
#CONFIG = fast mpi
#CONFIG = fast mpi openmp
#CONFIG = mpi debug
#CONFIG = debug
#CONFIG = fast noassert
#CONFIG = noassert snappy debug
#CONFIG = benchmark snappy debug openmp
#CONFIG = ultradebug

#CONFIG += c++
CONFIG += ansi
#CONFIG += c99


#LD_PRELOAD=$(pfx)/lib/valgrind/libmpiwrap-x86-linux.so \
#	mpirun -np 3 valgrind ./cli 2>&1 | tee out

#valgrind --num-callers=50 --db-attach=yes --db-command='cgdb -- %f %p'

ifeq ($(CC),g++)
	CONFIG += c++
endif

CFLAGS += -fwhole-program
CFLAGS += -Wall -Wextra
DFLAGS += -DINCLUDE_SOURCE


## Serious debugging is about to happen.
ifneq (,$(findstring ultradebug,$(CONFIG)))
	CONFIG := $(filter-out snappy fast debug,$(CONFIG))
	CFLAGS += -g3
endif
## Go really fast.
ifneq (,$(findstring fast,$(CONFIG)))
	CFLAGS += -O3
	#CFLAGS += -Ofast
	#CFLAGS += -march=native
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
	CC = $(CXX)
	#CFLAGS += -fno-rtti
	CFLAGS += -DCOMPILER_HAS_BOOL
endif
## Allow distributed parallelism.
ifneq (,$(findstring mpi,$(CONFIG)))
	ifneq (,$(findstring c++,$(CONFIG)))
		CC = mpicxx
	else
		CC = mpicc
	endif
	# OpenMPI headers currently don't play well with C89.
	CONFIG := $(filter-out ansi,$(CONFIG))
	CFLAGS += -DDistribCompute -DCompressBigCompute
	LFLAGS += -lz
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


CSources = bitstring.c \
		   kdtree.c \
		   material.c \
		   order.c \
		   pnm-image.c \
		   raytrace.c \
		   scene.c \
		   simplex.c \
		   slist.c \
		   space.c \
		   testcase.c \
		   util.c \
		   wavefront-file.c \
		   xfrm.c


LFLAGS += -lm

all: cli gui verify
	# Done!

OpenCLPath = /home/grencez/ati-stream-sdk-v2.3-lnx64
OpenCLLibPath = $(OpenCLPath)/lib/x86_64
hello: hello.c $(CSources)
	$(CC) $(CFLAGS) $(DFLAGS) -I $(OpenCLPath)/include $< -o $@ \
		-L $(OpenCLLibPath) -lOpenCL $(LFLAGS)

.PHONY: test-hello
test-hello: hello
	LD_LIBRARY_PATH=$(OpenCLLibPath) ./$<

cli: cli.c compute.c $(CSources)
	$(CC) $(CFLAGS) $(DFLAGS) $< -o $@ $(LFLAGS)

gui: gui.c compute.c motion.c $(CSources)
	$(CC) $(CFLAGS) $(DFLAGS) `pkg-config --cflags gtk+-2.0` \
		`sdl-config --cflags` \
		$< -o $@ \
		`pkg-config --libs gtk+-2.0` \
		-lSDL \
		$(LFLAGS)

verify: verif/main.c $(CSources)
	$(CC) $(CFLAGS) $(DFLAGS) -I . $< -o $@ $(LFLAGS)

.PHONY: test
test: cli
	#valgrind --track-origins=yes ./$<
	./$<

.PHONY: clean
clean:
	rm -f hello cli gui verify

