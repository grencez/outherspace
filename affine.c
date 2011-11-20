
#include "affine.h"

    void
identity_AffineMap (AffineMap* map)
{
    identity_PointXfrm (&map->xfrm);
    zero_Point (&map->xlat);
}

    void
mapvec_Point (Point* dst, const AffineMap* map, const Point* src)
{
    xfrm_Point (dst, &map->xfrm, src);
}

    void
map_Point (Point* dst, const AffineMap* map, const Point* src)
{
    Point u;
    xfrm_Point (&u, &map->xfrm, src);
    summ_Point (dst, &u, &map->xlat);
}

    void
map_Ray (Ray* dst, const AffineMap* map, const Ray* src)
{
    map_Point (&dst->origin, map, &src->origin);
    mapvec_Point (&dst->direct, map, &src->direct);
}

    /** Post-multiply by a scaling operation.**/
    void
scale0_AffineMap (AffineMap* map, real x)
{
    scale_PointXfrm (&map->xfrm, &map->xfrm, x);
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

