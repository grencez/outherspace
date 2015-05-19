
#ifndef PNM_IMAGE_H_
#define PNM_IMAGE_H_
#include "util.h"

void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems);
void output_PPM_image (const char* filename, uint nrows, uint ncols,
                       const byte* pixels);

byte*
readin_PPM_image (uint* ret_nrows, uint* ret_ncols,
                  const char* pathname, const char* filename);

#endif

