
#ifndef AFFINE_H_
#define AFFINE_H_
#include "xfrm.h"

typedef struct AffineMap AffineMap;

struct AffineMap
{
    PointXfrm xfrm;
    Point xlat;
};

void
identity_AffineMap (AffineMap* map);
void
mapvec_Point (Point* dst, const AffineMap* map, const Point* src);
void
map_Point (Point* dst, const AffineMap* map, const Point* src);
void
map_Ray (Ray* dst, const AffineMap* map, const Ray* src);

void
scale0_AffineMap (AffineMap* map, real x);
void
xlat0_AffineMap (AffineMap* map, const Point* a);
void
xfrm0_AffineMap (AffineMap* map, const PointXfrm* a);

#ifdef INCLUDE_SOURCE
#include "affine.c"
#endif
#endif

