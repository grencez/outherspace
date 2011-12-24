
#ifndef WAVEFRONT_FILE_H_
#define WAVEFRONT_FILE_H_
#include "scene.h"
#include "affine.h"

bool
output_wavefront (const Scene* scene,
                  const char* pathname,
                  const char* filename);
bool
readin_wavefront (Scene* scene, const char* pathname, const char* filename);

#ifdef INCLUDE_SOURCE
#include "wavefront-file.c"
#endif
#endif

