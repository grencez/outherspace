
#ifndef RAYTRACE_H_
#define RAYTRACE_H_

#include "kdtree.h"
#include "scene.h"
#include "xfrm.h"

struct ray_space_struct
{
    uint nelems;
    Triangle* elems;
    Scene scene;
    KDTree tree;
};
typedef struct ray_space_struct RaySpace;

void cleanup_RaySpace (RaySpace* space);
void rays_to_hits_fish (uint* hits, uint nrows, uint ncols,
                        const RaySpace* space, real zpos);
void rays_to_hits_from_point (uint* hits, uint nrows, uint ncols,
                              const RaySpace* space, real zpos);
void rays_to_hits_plane (uint* hits, uint nrows, uint ncols,
                         const RaySpace* space, real zpos);
void rays_to_hits (uint* hits, uint nrows, uint ncols,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis);

#include "raytrace.c"
#endif

