
#ifndef PNM_IMAGE_H_
#define PNM_IMAGE_H_
#include "raytrace.h"

void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PPM_image (const char* filename, uint nrows, uint ncols,
                       const byte* pixels);

byte*
readin_PPM_image (const char* filename, uint* ret_nrows, uint* ret_ncols);

#ifdef INCLUDE_SOURCE
#include "pnm-image.c"
#endif
#endif

