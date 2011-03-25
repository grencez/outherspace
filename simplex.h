
#ifndef SIMPLEX_H_
#ifndef __OPENCL_VERSION__
#define SIMPLEX_H_
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct plane_struct
{
    real offset;
    Point normal;
};
typedef struct plane_struct Plane;

#define NTrianglePoints 3
struct triangle_struct
{
    Point pts[NTrianglePoints];
};
typedef struct triangle_struct Triangle;

struct bary_simplex_struct
{
    Plane plane;
    Plane barys[NDimensions-1];
};
typedef struct bary_simplex_struct BarySimplex;

void init_Plane (Plane* plane, const Point* normal, const Point* point);
real distance_Plane (const Plane* plane, const Point* point);
void proj_Plane (Point* dst, const Point* a, const Plane* plane);
void barycentric_Plane (Plane* dst, const Plane* plane, const Point* point);
void init_BarySimplex (BarySimplex* elem, const PointXfrm* raw);
bool hit_BarySimplex (real* restrict ret_dist,
                      const Point* restrict origin,
                      const Point* restrict dir,
                      const BarySimplex* restrict elem);

#ifndef __OPENCL_VERSION__
void output_Triangle (FILE* out, const Triangle* elem);
#ifdef INCLUDE_SOURCE
#include "simplex.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

