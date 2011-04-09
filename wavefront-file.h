
#ifndef WAVEFRONT_FILE_H_
#define WAVEFRONT_FILE_H_
#include "raytrace.h"

bool readin_wavefront (RaySpace* space, const char* filename);

#ifdef INCLUDE_SOURCE
#include "wavefront-file.c"
#endif
#endif

