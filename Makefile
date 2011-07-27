
#CC = clang
#CC = gcc
#CC = llvm-gcc

DFLAGS += -DNDimensions=3

#CONFIG += c++
CONFIG += ansi
#CONFIG += c99
CONFIG += fast
#CONFIG += snappy
#CONFIG += debug
#CONFIG += ultradebug  # -fast -snappy -debug
CONFIG += openmp
#CONFIG += mpi  # -ansi
#CONFIG += trivialmpi  # +mpi
#CONFIG += profile
#CONFIG += benchmark  # +noassert
#CONFIG += noassert
#CONFIG += macapp


#LD_PRELOAD=$(pfx)/lib/valgrind/libmpiwrap-x86-linux.so \
#	mpirun -np 3 valgrind ./cli 2>&1 | tee out

#valgrind --num-callers=50 --db-attach=yes --db-command='cgdb -- %f %p'


ifeq ($(CC),g++)
	CONFIG += c++
endif

CFLAGS += -fwhole-program
CFLAGS += -Wall -Wextra
DFLAGS += -DINCLUDE_SOURCE

ifneq (,$(filter macapp,$(CONFIG)))
	# Everything on this OS X setup is 32 bit.
	CFLAGS += -m32
	DFLAGS += -DRunFromMyMac
	GuiCFlags += -I$(HOME)/gtk/inst/include \
				 -I/Library/Frameworks/SDL.framework/Headers \
				 -I/Library/Frameworks/SDL_mixer.framework/Headers
	GuiLFlags += -framework SDL -framework SDL_mixer
else
	GuiCFlags += $(shell pkg-config --cflags sdl)
	GuiLFlags += $(shell pkg-config --libs sdl) -lSDL_mixer
endif

## Serious debugging is about to happen.
ifneq (,$(filter ultradebug,$(CONFIG)))
	CONFIG := $(filter-out snappy fast debug,$(CONFIG))
	CFLAGS += -g3
endif
## Go really fast.
ifneq (,$(filter fast,$(CONFIG)))
	CFLAGS += -O3
	#CFLAGS += -Ofast
	#CFLAGS += -march=native
	#CFLAGS += -ffast-math
	#CFLAGS += -march=native -mtune=native
endif
## Go pretty fast.
ifneq (,$(filter snappy,$(CONFIG)))
	CFLAGS += -O2
endif
## Add debugging symbols.
ifneq (,$(filter debug,$(CONFIG)))
	CFLAGS += -g
endif
ifneq (,$(filter profile,$(CONFIG)))
	CFLAGS += -pg
	LFLAGS += -pg
endif

## Enable benchmarking. (take out file writes)
ifneq (,$(filter benchmark,$(CONFIG)))
	CFLAGS += -DBENCHMARKING
	CONFIG += noassert
endif
## Disable assertions.
ifneq (,$(filter noassert,$(CONFIG)))
	CFLAGS += -DNDEBUG
endif
## Do we have bool type?
ifneq (,$(filter c++,$(CONFIG)))
	CC = $(CXX)
	#CFLAGS += -fno-rtti
	CFLAGS += -DCOMPILER_HAS_BOOL
endif
## Allow distributed parallelism.
ifneq (,$(filter mpi trivialmpi,$(CONFIG)))
	ifneq (,$(filter c++,$(CONFIG)))
		CC = mpicxx
	else
		CC = mpicc
	endif
	# OpenMPI headers currently don't play well with C89.
	CONFIG := $(filter-out ansi,$(CONFIG))
	DFLAGS += -DDistribCompute
	# Use the ray tracer that disregards load balancing and compression.
	ifneq (,$(filter trivialmpi,$(CONFIG)))
		DFLAGS += -DTrivialMpiRayTrace
	else
		DFLAGS += -DCompressBigCompute
		LFLAGS += -lz
	endif
endif
## Use the C99 standard.
ifneq (,$(filter c99,$(CONFIG)))
	CFLAGS += -std=c99
endif
## Stick to the ANSI standard.
ifneq (,$(filter ansi,$(CONFIG)))
	CFLAGS += -ansi -pedantic
endif
## Allow parallelism.
ifneq (,$(filter openmp,$(CONFIG)))
	CFLAGS += -fopenmp
endif


CSources = bitstring.c \
		   kdtree.c \
		   kptree.c \
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

# Note: OpenCL code does not function at this time, don't bother.
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
	$(CC) $(CFLAGS) $(DFLAGS) \
		$(GuiCFlags) \
	   	`pkg-config --cflags gtk+-2.0` \
		$< -o $@ \
		`pkg-config --libs gtk+-2.0` \
		`pkg-config --libs gthread-2.0` \
		$(GuiLFlags) $(LFLAGS)

verify: verif/main.c $(CSources)
	$(CC) $(CFLAGS) $(DFLAGS) -I . $< -o $@ $(LFLAGS)

.PHONY: test
test: cli
	#valgrind --track-origins=yes ./$<
	./$<

.PHONY: clean
clean:
	rm -f hello cli gui verify

