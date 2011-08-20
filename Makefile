
#CC = clang
#CC = gcc
#CC = llvm-gcc
#CC = icc

DFLAGS += -DNDimensions=3

CONFIG += image
CONFIG += sound

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


# For ICC:
#   source $HOME/local/stow/icc-2011.5.220/bin/iccvars.sh intel64


#LD_PRELOAD=$(pfx)/lib/valgrind/libmpiwrap-x86-linux.so \
#	mpirun -np 3 valgrind ./cli 2>&1 | tee out

#valgrind --num-callers=50 --db-attach=yes --db-command='cgdb -- %f %p'


ifeq ($(CC),g++)
	CONFIG += c++
endif

ifeq ($(CC),icc)
	#CFLAGS += -Wremarks
else
	CFLAGS += -fwhole-program
	CFLAGS += -Wall -Wextra
endif

DFLAGS += -DINCLUDE_SOURCE

ifneq (,$(filter image,$(CONFIG)))
	GuiDFlags += -DSupportImage
endif
ifneq (,$(filter sound,$(CONFIG)))
	GuiDFlags += -DSupportSound
endif

ifneq (,$(filter macapp,$(CONFIG)))
	# Everything on this OS X setup is 32 bit.
	CFLAGS += -m32
	DFLAGS += -DRunFromMyMac
	GuiCFlags += -I/Library/Frameworks/SDL.framework/Headers
	GuiLFlags += -framework SDL
	ifneq (,$(filter image,$(CONFIG)))
		GuiCFlags += -I/Library/Frameworks/SDL_image.framework/Headers
		GuiLFlags += -framework SDL_image
	endif
	ifneq (,$(filter sound,$(CONFIG)))
		GuiCFlags += -I/Library/Frameworks/SDL_mixer.framework/Headers
		GuiLFlags += -framework SDL_mixer
	endif
else
	GuiCFlags += $(shell pkg-config --cflags sdl)
	GuiLFlags += $(shell pkg-config --libs sdl)
	ifneq (,$(filter image,$(CONFIG)))
		GuiLFlags += -lSDL_image
	endif
	ifneq (,$(filter sound,$(CONFIG)))
		GuiLFlags += -lSDL_mixer
	endif
endif

## Serious debugging is about to happen.
ifneq (,$(filter ultradebug,$(CONFIG)))
	CONFIG := $(filter-out snappy fast debug,$(CONFIG))
	CFLAGS += -g3
endif
## Go really fast.
ifneq (,$(filter fast,$(CONFIG)))
	CFLAGS += -O3
	#CFLAGS += -ftree-vectorizer-verbose=2
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
	ifeq ($(CC),icc)
	  CFLAGS += -openmp
	else
	  CFLAGS += -fopenmp
	endif
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

default: cli gui verify
	# Done!

all: imgdiff cli gui verify
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

GuiCFlags += $(CFLAGS)
GuiDFlags += $(DFLAGS)
GuiLFlags += $(LFLAGS)
gui: gui.c gui-indep.c compute.c motion.c $(CSources)
	$(CC) $(GuiCFlags) $(GuiDFlags) $< -o $@ $(GuiLFlags)


VerifyCFlags := $(filter-out -fwhole-program,$(CFLAGS))
VerifyDFlags := $(filter-out -DINCLUDE_SOURCE,$(DFLAGS))

verify: verif/main.c $(CSources)
	$(CC) $(VerifyCFlags) $(VerifyDFlags) -I . $^ -o $@ $(LFLAGS)

imgdiff: imgdiff.c pnm-image.c
	$(CC) $(CFLAGS) $(DFLAGS) -I . $< -o $@ $(LFLAGS)

.PHONY: test
test: cli
	#valgrind --track-origins=yes ./$<
	./$<

.PHONY: clean
clean:
	rm -f hello cli gui verify imgdiff

