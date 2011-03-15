
#ifndef MAIN_H_
#define MAIN_H_
#include "raytrace.h"

void random_RaySpace (RaySpace* space, uint nelems);
void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PPM_image (const char* filename, uint nrows, uint ncols,
                       const byte* pixels);
bool readin_wavefront (RaySpace* space, const char* filename);

#ifdef INCLUDE_SOURCE
#include "main.c"
#endif
#endif

