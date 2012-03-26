
#ifndef Point_H_
#ifndef __OPENCL_VERSION__
#define Point_H_
#include "space.h"
#include "space-junk.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

void
copy_Point (Point* dst, const Point* src);
void
diff_Point (Point* dst, const Point* a, const Point* b);
real
dot_Point (const Point* a, const Point* b);
void
summ_Point (Point* dst, const Point* a, const Point* b);
void
prod_Point (Point* dst, const Point* a, const Point* b);
void
quot_Point (Point* dst, const Point* a, const Point* b);
void
scale_Point (Point* dst, const Point* a, real k);
void
reci_Point (Point* dst, const Point* src);
void
quot1_Point (Point* dst, const Point* src, real x);
void
follow_Point (Point* dst, const Point* origin, const Point* direct, real mag);
void
follow_Ray (Point* isect, const Ray* ray, real mag);
void
zero_Point (Point* a);
bool
equal_Point (const Point* a, const Point* b);
bool
ordered_Point (const Point* a, const Point* b);
void
checker_negate_Point (Point* p);
real
magnitude_Point (const Point* a);
real
taximag_Point (const Point* a);
real
dist_Point (const Point* a, const Point* b);
void
normalize_Point (Point* dst, const Point* a);
void
proj_Point (Point* dst, const Point* a, const Point* b);
void
orth_Point (Point* dst, const Point* a, const Point* b);
void
proj_unit_Point (Point* dst, const Point* a, const Point* b);
void
orth_unit_Point (Point* dst, const Point* a, const Point* b);
void
reflect_Point (Point* refl, const Point* p,
               const Point* normal, real dot);

#ifdef IncludeC
#include "point.c"
#endif

#endif

