
#ifndef SPACE_H_
#ifndef __OPENCL_VERSION__
#define SPACE_H_
#include "util.h"
#include <stdio.h>
#endif  /* #ifndef __OPENCL_VERSION__ */


#define NDimensions 3

struct point_struct
{
    real coords[NDimensions];
};
typedef struct point_struct Point;

#define NTrianglePoints 3
struct triangle_struct
{
    Point pts[NTrianglePoints];
};
typedef struct triangle_struct Triangle;

struct bounding_box_struct
{
    Point min_corner;
    Point max_corner;
};
typedef struct bounding_box_struct BoundingBox;

void diff_Point (Point* dst, const Point* a, const Point* b);
real dot_Point (const Point* a, const Point* b);
void summ_Point (Point* dst, const Point* a, const Point* b);
void scale_Point (Point* dst, const Point* a, real k);
void zero_Point (Point* a);
void copy_Point (Point* dst, const Point* src);
void copy_BoundingBox (BoundingBox* dst, const BoundingBox* src);
real magnitude_Point (const Point* a);
void normalize_Point (Point* dst, const Point* a);
void proj_Point (Point* dst, const Point* a, const Point* b);
tristate facing_BoundingPlane (uint dim, real plane,
                               const Point* origin, const Point* dir);
bool hit_inner_BoundingPlane (Point* entrance,
                              uint dim, real plane,
                              __global const BoundingBox* box,
                              const Point* origin, const Point* dir);
bool hit_outer_BoundingBox (Point* entrance,
                            __global const BoundingBox* box,
                            const Point* origin, const Point* dir);
bool hit_BoundingBox (Point* entrance,
                      const BoundingBox* box,
                      const Point* origin, const Point* dir);
void init_BoundingBox (BoundingBox* box, uint npoints, const Point* points);
void adjust_BoundingBox (BoundingBox* box, const Point* point);
bool inside_BoundingBox (__global const BoundingBox* box, const Point* point);
real surface_area_BoundingBox (const BoundingBox* box);
void split_BoundingBox (BoundingBox* lo_box, BoundingBox* hi_box,
                        const BoundingBox* box,
                        uint split_dim, real split_pos);

#ifndef __OPENCL_VERSION__
void output_Point (FILE* out, const Point* point);
void output_BoundingBox (FILE* out, const BoundingBox* box);
void output_Triangle (FILE* out, const Triangle* elem);
#ifdef INCLUDE_SOURCE
#include "space.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

