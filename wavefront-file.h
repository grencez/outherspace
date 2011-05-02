
#ifndef WAVEFRONT_FILE_H_
#define WAVEFRONT_FILE_H_
#include "scene.h"

bool
readin_wavefront (Scene* scene, const char* filename);

#ifdef INCLUDE_SOURCE
#include "wavefront-file.c"
#endif
#endif

