
#ifndef RAYTRACE_H_
#ifndef __OPENCL_VERSION__
#define RAYTRACE_H_

#include "kdtree.h"
#include "scene.h"
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct ray_space_struct
{
    uint nelems;
    Triangle* elems;
    Scene scene;
    KDTree tree;
};
typedef struct ray_space_struct RaySpace;

struct ray_cast_params_struct
{
    Point origin;
    Point dir_start;
    Point dir_delta[2];
    BoundingBox box;
    uint nelems;
    uint npixels[2];
    bool inside_box;
};
typedef struct ray_cast_params_struct RayCastParams;

#ifndef __OPENCL_VERSION__
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

void build_RayCastParams (RayCastParams* params,
                          uint nrows, uint ncols,
                          const RaySpace* space,
                          const Point* origin,
                          const PointXfrm* view_basis);

void cleanup_RaySpace (RaySpace* space);

#include "raytrace.c"
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

