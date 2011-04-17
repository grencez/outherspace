
#ifndef SIMPLEX_H_
#ifndef __OPENCL_VERSION__
#define SIMPLEX_H_
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

#define NTrianglePoints 3
struct triangle_struct
{
    Point pts[NTrianglePoints];
};
typedef struct triangle_struct Triangle;

struct plane_struct
{
    real offset;
    Point normal;
};
typedef struct plane_struct Plane;

struct bary_point_struct
{
    real coords[NDimensions-1];
};
typedef struct bary_point_struct BaryPoint;

struct bary_simplex_struct
{
    Plane plane;
    Plane barys[NDimensions-1];
};
typedef struct bary_simplex_struct BarySimplex;


bool
hit_Triangle (real* restrict ret_dist,
              const Point* restrict origin,
              const Point* restrict dir,
              const Triangle* restrict elem);
bool hit_proj_Triangle (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Triangle* restrict elem,
                        const PointXfrm* restrict view_basis);
bool hit_weak_Triangle (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Triangle* restrict elem);

void init_Plane (Plane* plane, const Point* normal, const Point* point);
real distance_Plane (const Plane* plane, const Point* point);
void proj_Plane (Point* dst, const Point* a, const Plane* plane);
void barycentric_Plane (Plane* dst, const Plane* plane, const Point* point);

void init_BarySimplex (BarySimplex* elem, const PointXfrm* raw);
bool
hit_BarySimplex (real* restrict ret_dist,
                 const Point* restrict origin,
                 const Point* restrict dir,
                 const BarySimplex* restrict elem);

void
tri_to_BarySimplex (BarySimplex* simplex, const Triangle* tri);

#ifndef __OPENCL_VERSION__
void output_Triangle (FILE* out, const Triangle* elem);
#ifdef INCLUDE_SOURCE
#include "simplex.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

