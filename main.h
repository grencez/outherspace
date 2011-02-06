
#ifndef MAIN_H_
#define MAIN_H_
#include "raytrace.h"

void random_RaySpace (RaySpace* space, uint nelems);
void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
bool readin_wavefront (RaySpace* space, const char* filename);

#include "main.c"
#endif

