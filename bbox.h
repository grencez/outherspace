
#ifndef BBox_H_
#ifndef __OPENCL_VERSION__
#define BBox_H_
#include "space.h"
#include "util.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

void
zero_BBox (BBox* box);
tristate facing_BoundingPlane (uint dim, real plane,
                               const Point* origin, const Point* dir);
bool
hit_inner_BoundingPlane (Point* entrance,
                         uint dim, real plane,
                         __global const BBox* box,
                         const Point* origin,
                         const Point* dir);
real
hit_inner_BBox (Point* isect,
                uint* ret_dim,
                const BBox* box,
                const Ray* ray,
                const Point* invdirect);
bool hit_outer_BBox (Point* entrance,
                     __global const BBox* box,
                     const Point* origin, const Point* dir);
bool hit_BBox (Point* entrance,
               const BBox* box,
               const Point* origin, const Point* dir);
void init0_BBox (BBox* box);
void init_BBox (BBox* box, uint npoints, const Point* points);
void adjust_BBox (BBox* box, const Point* point);
void
include_BBox (BBox* dst, const BBox* a, const BBox* b);
void
centroid_BBox (Point* dst, const BBox* box);
void
measure_BBox (Point* dst, const BBox* box);
bool inside_BBox (__global const BBox* box, const Point* point);
real surface_area_BBox (const BBox* box);
void split_BBox (BBox* lo_box, BBox* hi_box,
                 const BBox* box,
                 uint split_dim, real split_pos);
void
merge_BBox (BBox* dst, const BBox* a, const BBox* b);
void
clip_BBox (BBox* dst, const BBox* a, const BBox* b);
void
trxfrm_BBox (BBox* dst,
             const PointXfrm* basis,
             const BBox* box,
             const Point* new_centroid);

#ifdef IncludeC
#include "bbox.c"
#endif

#endif

