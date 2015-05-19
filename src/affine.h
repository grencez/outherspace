
#ifndef IAMap_H_
#ifndef __OPENCL_VERSION__
#define IAMap_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

#if 0
    /**
     * map (x) = xlat + xfrm scale x
     * map^-1 (x) = (xfrm^-1 (x - xlat)) / scale
     **/
struct IAMap
{
    PointXfrm xfrm;
    Point xlat;
    Point scale;
};
#endif

void
follow_Ray (Point* isect, const Ray* ray, real mag);

void
identity_IAMap (IAMap* map);
void
normalize_IAMap (IAMap* map);
void
mapo_Point (Point* dst, const IAMap* map, const Point* src);
void
mapvec_Point (Point* dst, const IAMap* map, const Point* src);
void
map_Point (Point* dst, const IAMap* map, const Point* src);
void
map_Ray (Ray* dst, const IAMap* map, const Ray* src);

void
imapvec_Point (Point* dst, const IAMap* map, const Point* src);
void
imap_Point (Point* dst, const IAMap* map, const Point* src);
void
imap_Ray (Ray* dst, const IAMap* map, const Ray* src);

void
scale0_IAMap (IAMap* map, real x);
void
prod0_IAMap (IAMap* map, const Point* a);
void
xlat0_IAMap (IAMap* map, const Point* a);
void
xfrm0_IAMap (IAMap* map, const PointXfrm* a);
void
xlat_IAMap (IAMap* B, const Point* x, const IAMap* A);
void
xfrm_IAMap (IAMap* C, const PointXfrm* A, const IAMap* B);

#endif

