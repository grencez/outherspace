
#ifndef Simplex_H_
#ifndef __OPENCL_VERSION__
#define Simplex_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */


bool
hit_Simplex (real* restrict ret_dist,
             const Ray ray,
             const Simplex elem,
             Trit front);

void init_Plane (Plane* plane, const Point* normal, const Point* point);
real
dist_Plane (const Plane* plane, const Point* point);
void proj_Plane (Point* dst, const Point* a, const Plane* plane);
void barycentric_Plane (Plane* dst, const Plane* plane, const Point* point);
void
barycentric_Point (Point* bpoint, const Point* isect,
                   const BarySimplex* simplex);

real
area_Simplex (const Simplex* simplex);
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
                 const BarySimplex* restrict elem,
                 Trit front);


#endif

