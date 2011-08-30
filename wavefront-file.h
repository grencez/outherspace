
#ifndef WAVEFRONT_FILE_H_
#define WAVEFRONT_FILE_H_
#include "scene.h"

void
fixup_wavefront_Scene (Scene* scene);
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

