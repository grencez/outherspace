
#ifndef BBOX_H_
#ifndef __OPENCL_VERSION__
#define BBOX_H_
#include "space.h"
#include "util.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

void
zero_BoundingBox (BoundingBox* box);
void copy_BoundingBox (BoundingBox* dst, const BoundingBox* src);
void negate_Point (Point* dst, const Point* src);
tristate facing_BoundingPlane (uint dim, real plane,
                               const Point* origin, const Point* dir);
bool
hit_inner_BoundingPlane (Point* entrance,
                         uint dim, real plane,
                         __global const BoundingBox* box,
                         const Point* origin,
                         const Point* dir);
real
hit_inner_BoundingBox (Point* isect,
                       uint* ret_dim,
                       const BoundingBox* box,
                       const Ray* ray,
                       const Point* invdirect);
bool hit_outer_BoundingBox (Point* entrance,
                            __global const BoundingBox* box,
                            const Point* origin, const Point* dir);
bool hit_BoundingBox (Point* entrance,
                      const BoundingBox* box,
                      const Point* origin, const Point* dir);
void init_BoundingBox (BoundingBox* box, uint npoints, const Point* points);
void adjust_BoundingBox (BoundingBox* box, const Point* point);
void
include_BoundingBox (BoundingBox* dst,
                     const BoundingBox* a, const BoundingBox* b);
void
centroid_BoundingBox (Point* dst, const BoundingBox* box);
void
measure_BoundingBox (Point* dst, const BoundingBox* box);
bool inside_BoundingBox (__global const BoundingBox* box, const Point* point);
real surface_area_BoundingBox (const BoundingBox* box);
void split_BoundingBox (BoundingBox* lo_box, BoundingBox* hi_box,
                        const BoundingBox* box,
                        uint split_dim, real split_pos);
void
merge_BoundingBox (BoundingBox* dst,
                   const BoundingBox* a,
                   const BoundingBox* b);
void
clip_BoundingBox (BoundingBox* dst,
                  const BoundingBox* a,
                  const BoundingBox* b);
void
trxfrm_BoundingBox (BoundingBox* dst,
                    const PointXfrm* basis,
                    const BoundingBox* box,
                    const Point* new_centroid);

#ifdef IncludeC
#include "bbox.c"
#endif

#endif

