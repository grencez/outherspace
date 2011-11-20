
#ifndef SIMPLEX_H_
#ifndef __OPENCL_VERSION__
#define SIMPLEX_H_
#include "xfrm.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct Simplex Simplex;
typedef struct Plane Plane;
typedef struct BaryPoint BaryPoint;
typedef struct BarySimplex BarySimplex;

struct Simplex
{
    Point pts[NDimensions];
};

struct Plane
{
    real offset;
    Point normal;
};

struct BaryPoint
{
    real coords[NDimensions-1];
};

struct BarySimplex
{
    Plane plane;
    Plane barys[NDimensions-1];
};


bool
hit_Simplex (real* restrict ret_dist,
             const Ray ray,
             const Simplex elem);
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
void
barycentric_Point (Point* bpoint, const Point* isect,
                   const BarySimplex* simplex);

bool
degenerate_Simplex (const Simplex* raw);
bool
init_BarySimplex (BarySimplex* elem, const Simplex* raw);
bool
hit_Plane (real* restrict ret_dist,
           const Point* restrict origin,
           const Point* restrict direct,
           const Plane* restrict plane);
bool
hit_BarySimplex (real* restrict ret_dist,
                 const Ray* restrict ray,
                 const BarySimplex* restrict elem);

#ifndef __OPENCL_VERSION__
void output_Simplex (FILE* out, const Simplex* elem);
#ifdef INCLUDE_SOURCE
#include "simplex.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

