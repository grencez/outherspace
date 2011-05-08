
#ifndef SIMPLEX_H_
#ifndef __OPENCL_VERSION__
#define SIMPLEX_H_
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct simplex_struct
{
    Point pts[NDimensions];
};
typedef struct simplex_struct Simplex;

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
hit_Simplex (real* restrict ret_dist,
             const Point* restrict origin,
             const Point* restrict dir,
             const Simplex* restrict elem);
bool hit_proj_Simplex (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Simplex* restrict elem,
                        const PointXfrm* restrict view_basis);
bool hit_weak_Simplex (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Simplex* restrict elem);

void init_Plane (Plane* plane, const Point* normal, const Point* point);
real distance_Plane (const Plane* plane, const Point* point);
void proj_Plane (Point* dst, const Point* a, const Plane* plane);
void barycentric_Plane (Plane* dst, const Plane* plane, const Point* point);

bool
degenerate_Simplex (const Simplex* raw);
bool
init_BarySimplex (BarySimplex* elem, const Simplex* raw);
bool
hit_BarySimplex (real* restrict ret_dist,
                 const Point* restrict origin,
                 const Point* restrict dir,
                 const BarySimplex* restrict elem);

#ifndef __OPENCL_VERSION__
void output_Simplex (FILE* out, const Simplex* elem);
#ifdef INCLUDE_SOURCE
#include "simplex.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

