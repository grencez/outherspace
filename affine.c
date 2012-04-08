
#include "affine.h"

#include "point.h"
#include "xfrm.h"

    void
follow_Ray (Point* isect, const Ray* ray, real mag)
{
    follow_Point (isect, &ray->origin, &ray->direct, mag);
}

    void
identity_IAMap (IAMap* map)
{
    identity_PointXfrm (&map->xfrm);
    zero_Point (&map->xlat);
    set_Point (&map->scale, 1);
}

    void
normalize_IAMap (IAMap* map)
{
    PointXfrm A;
    orthonormalize_PointXfrm (&A, &map->xfrm);
    map->xfrm = A;
}

    /** Map a vector x without scaling: xfrm x **/
    void
mapo_Point (Point* dst, const IAMap* map, const Point* src)
{
    xfrm_Point (dst, &map->xfrm, src);
}

    /** Map a vector x: scale xfrm x **/
    void
mapvec_Point (Point* dst, const IAMap* map, const Point* src)
{
    Point u;
    prod_Point (&u, src, &map->scale);
    mapo_Point (dst, map, &u);
}

    /** Map a vector x: xlat + scale xfrm x **/
    void
map_Point (Point* dst, const IAMap* map, const Point* src)
{
    Point u;
    mapvec_Point (&u, map, src);
    summ_Point (dst, &map->xlat, &u);
}

    void
map_Ray (Ray* dst, const IAMap* map, const Ray* src)
{
    map_Point (&dst->origin, map, &src->origin);
    mapo_Point (&dst->direct, map, &src->direct);
}

    void
imapo_Point (Point* dst, const IAMap* map, const Point* src)
{
    trxfrm_Point (dst, &map->xfrm, src);
}

    void
imapvec_Point (Point* dst, const IAMap* map, const Point* src)
{
    Point u;
    imapo_Point (&u, map, src);
    quot_Point (dst, &u, &map->scale);
}

    void
imap_Point (Point* dst, const IAMap* map, const Point* src)
{
    Point u;
    diff_Point (&u, src, &map->xlat);
    imapvec_Point (dst, map, &u);
}

    void
imap_Ray (Ray* dst, const IAMap* map, const Ray* src)
{
    imap_Point (&dst->origin, map, &src->origin);
    imapo_Point (&dst->direct, map, &src->direct);
}

    /** Post-multiply by a scaling operation.**/
    void
scale0_IAMap (IAMap* map, real x)
{
    scale_Point (&map->scale, &map->scale, x);
}

    /** Post-multiply by an element-wise product.**/
    void
prod0_IAMap (IAMap* map, const Point* a)
{
    prod_Point (&map->scale, &map->scale, a);
}

    /** Post-multiply by a translate operation.**/
    void
xlat0_IAMap (IAMap* map, const Point* a)
{
    map_Point (&map->xlat, map, a);
}

    /** Post-multiply by a general linear transform operation.**/
    void
xfrm0_IAMap (IAMap* map, const PointXfrm* a)
{
    PointXfrm tmp;
    uint i;
    UFor( i, NDims )
        prod_Point (&tmp.pts[i], &map->scale, &a->pts[i]);
    xfrm_PointXfrm (&map->xfrm, &map->xfrm, &tmp);
}

    void
xlat_IAMap (IAMap* B, const Point* x, const IAMap* A)
{
    *B = *A;
    summ_Point (&B->xlat, &B->xlat, x);
}

    void
xfrm_IAMap (IAMap* C, const PointXfrm* A, const IAMap* B)
{
    C->scale = B->scale;
    xfrm_Point (&C->xlat, A, &B->xlat);
    xfrm_PointXfrm (&C->xfrm, A, &B->xfrm);
}

