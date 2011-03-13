
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

struct multi_ray_cast_params_struct
{
    Point origin;
    Point dir_start;
    Point dir_delta[2];
    BoundingBox box;
    uint nelems;
    uint npixels[2];
    bool inside_box;
};
typedef struct multi_ray_cast_params_struct MultiRayCastParams;

void dir_from_MultiRayCastParams (Point* dir, uint row, uint col,
                                  const MultiRayCastParams* params);

#ifndef __OPENCL_VERSION__
void rays_to_hits_fish (uint* hits, real* mags,
                        uint nrows, uint ncols,
                        const RaySpace* space, real zpos);
void rays_to_hits_from_point (uint* hits, real* mags,
                              uint nrows, uint ncols,
                              const RaySpace* space, real zpos);
void rays_to_hits_plane (uint* hits, real* mags,
                         uint nrows, uint ncols,
                         const RaySpace* space, real zpos);

void setup_ray_pixel_deltas (Point* dir_start,
                             Point* row_delta,
                             Point* col_delta,
                             uint nrows, uint ncols,
                             const PointXfrm* view_basis,
                             real view_angle);
void rays_to_hits (uint* hits, real* mags,
                   uint nrows, uint ncols,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis,
                   real view_angle);

void build_MultiRayCastParams (MultiRayCastParams* params,
                               uint nrows, uint ncols,
                               const RaySpace* space,
                               const Point* origin,
                               const PointXfrm* view_basis,
                               real view_angle);

void cleanup_RaySpace (RaySpace* space);
#ifdef INCLUDE_SOURCE
#include "raytrace.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

