
#ifndef SPACE_H_
#ifndef __OPENCL_VERSION__
#define SPACE_H_
#include "util.h"
#include <stdio.h>
#include "space-junk.h"
#endif  /* #ifndef __OPENCL_VERSION__ */


#ifndef NDimensions
#define NDimensions 3
#endif

typedef struct Point Point;
typedef struct BoundingBox BoundingBox;

struct Point
{
    real coords[NDimensions];
};

struct BoundingBox
{
    Point min;
    Point max;
};

void diff_Point (Point* dst, const Point* a, const Point* b);
real dot_Point (const Point* a, const Point* b);
void summ_Point (Point* dst, const Point* a, const Point* b);
void scale_Point (Point* dst, const Point* a, real k);
void
invmul_Point (Point* dst, const Point* src);
void zero_Point (Point* a);
bool
equal_Point (const Point* a, const Point* b);
bool
ordered_Point (const Point* a, const Point* b);
void
zero_BoundingBox (BoundingBox* box);
void copy_Point (Point* dst, const Point* src);
void copy_BoundingBox (BoundingBox* dst, const BoundingBox* src);
void negate_Point (Point* dst, const Point* src);
void
checker_negate_Point (Point* p);
real magnitude_Point (const Point* a);
void normalize_Point (Point* dst, const Point* a);
void proj_Point (Point* dst, const Point* a, const Point* b);
void orth_Point (Point* dst, const Point* a, const Point* b);
void
proj_unit_Point (Point* dst, const Point* a, const Point* b);
void
orth_unit_Point (Point* dst, const Point* a, const Point* b);
void
reflect_Point (Point* refl, const Point* p,
               const Point* normal, real dot);
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
                       const Point* origin,
                       const Point* direct,
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

#ifndef __OPENCL_VERSION__
void output_Point (FILE* out, const Point* point);
void output_BoundingBox (FILE* out, const BoundingBox* box);
#ifdef INCLUDE_SOURCE
#include "space.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

