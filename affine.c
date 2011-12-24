
#include "affine.h"

    void
identity_AffineMap (AffineMap* map)
{
    identity_PointXfrm (&map->xfrm);
    zero_Point (&map->xlat);
    map->scale = 1;
}

    /** Map a vector x without scaling: xfrm x **/
    void
mapo_Point (Point* dst, const AffineMap* map, const Point* src)
{
    xfrm_Point (dst, &map->xfrm, src);
}

    /** Map a vector x: scale xfrm x **/
    void
mapvec_Point (Point* dst, const AffineMap* map, const Point* src)
{
    Point u;
    mapo_Point (&u, map, src);
    scale_Point (dst, &u, map->scale);
}

    /** Map a vector x: xlat + scale xfrm x **/
    void
map_Point (Point* dst, const AffineMap* map, const Point* src)
{
    Point u;
    mapvec_Point (&u, map, src);
    summ_Point (dst, &map->xlat, &u);
}

    void
map_Ray (Ray* dst, const AffineMap* map, const Ray* src)
{
    map_Point (&dst->origin, map, &src->origin);
    mapo_Point (&dst->direct, map, &src->direct);
}

    void
invmapo_Point (Point* dst, const AffineMap* map, const Point* src)
{
    trxfrm_Point (dst, &map->xfrm, src);
}

    void
invmapvec_Point (Point* dst, const AffineMap* map, const Point* src)
{
    Point u;
    invmapo_Point (&u, map, src);
    quot_Point (dst, &u, map->scale);
}

    void
invmap_Point (Point* dst, const AffineMap* map, const Point* src)
{
    Point u;
    diff_Point (&u, src, &map->xlat);
    invmapvec_Point (dst, map, &u);
}

    void
invmap_Ray (Ray* dst, const AffineMap* map, const Ray* src)
{
    invmap_Point (&dst->origin, map, &src->origin);
    invmapo_Point (&dst->direct, map, &src->direct);
}

    /** Post-multiply by a scaling operation.**/
    void
scale0_AffineMap (AffineMap* map, real x)
{
    map->scale *= x;
}

    /** Post-multiply by a translate operation.**/
    void
xlat0_AffineMap (AffineMap* map, const Point* a)
{
    map_Point (&map->xlat, map, a);
}

    /** Post-multiply by a general linear transform operation.**/
    void
xfrm0_AffineMap (AffineMap* map, const PointXfrm* a)
{
    xfrm_PointXfrm (&map->xfrm, &map->xfrm, a);
}

