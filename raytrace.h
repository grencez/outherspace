
#ifndef RAYTRACE_H_
#define RAYTRACE_H_

#include "kdtree.h"

struct ray_space_struct
{
    uint nelems;
    Triangle* selems;
    const Triangle** elems;
    KDTree tree;
};
typedef struct ray_space_struct RaySpace;

void cleanup_RaySpace (RaySpace* space);
void rays_to_hits_from_point (uint* hits, uint nrows, uint ncols,
                              uint nelems, const Triangle* elems,
                              const KDTree* space, real zpos);
void rays_to_hits (uint* hits, uint nrows, uint ncols,
                   uint nelems, const Triangle* elems,
                   const KDTree* space, real zpos);

#include "raytrace.c"
#endif
