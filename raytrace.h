
#ifndef RAYTRACE_H_
#define RAYTRACE_H_

#include "kdtree.c"

void rays_to_hits (uint* hits, uint nrows, uint ncols,
                   uint nelems, const Triangle* elems,
                   const KDTree* space);

#endif

