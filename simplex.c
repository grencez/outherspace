
#ifndef __OPENCL_VERSION__
#include "simplex.h"

#include "affine.h"
#include "point.h"
#include "space-junk.h"
#include "util.h"
#include "xfrm.h"

#include <assert.h>
#include <math.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

    /* But really only works for 3D!*/
static
    void
cross_Point (Point* restrict dst,
             const Point* restrict a,
             const Point* restrict b)
{
    uint i;
    dst->coords[0] = a->coords[1] * b->coords[2] - a->coords[2] * b->coords[1];
    dst->coords[1] = a->coords[2] * b->coords[0] - a->coords[0] * b->coords[2];
    dst->coords[2] = a->coords[0] * b->coords[1] - a->coords[1] * b->coords[0];
    for (i = 3; i < NDimensions; ++i)
        dst->coords[i] = 0;
}

    bool
hit_Simplex (real* restrict ret_dist,
             const Ray ray,
             const Simplex elem,
             Trit front)
{
  const real fuzz = 128 * Epsilon_real;
        /* const real epsilon = 16 * Epsilon_real; */
    real epsilon = fuzz;
    Point edge1, edge2, tvec, pvec, qvec;
    real det, inv_det;
    real u, v;

    Op_Point_200( &tvec  ,-, &ray.origin  , &elem.pts[0] );
    Op_Point_200( &edge2 ,-, &elem.pts[2] , &elem.pts[0] );
    Op_Point_200( &edge1 ,-, &elem.pts[1] , &elem.pts[0] );
    cross_Point (&pvec, &ray.direct, &edge2);
    u = dot_Point (&tvec, &pvec);
    det = dot_Point (&edge1, &pvec);

    if (det > epsilon && front != Nil)
    {
        if (u < 0 || u > det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (&ray.direct, &qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon && front != Yes)
    {
        if (u > 0 || u < det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (&ray.direct, &qvec);
        if (v > 0 || u + v < det)
            return false;
    }
    else
    {
        return false;
    }

    inv_det = 1 / det;
    *ret_dist = dot_Point (&edge2, &qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return *ret_dist >= 0;
}

void init_Plane (Plane* plane, const Point* normal, const Point* point)
{
    normalize_Point (&plane->normal, normal);
    plane->offset = - dot_Point (&plane->normal, point);
}


    /** Distance from the plane to a point.**/
    real
dist_Plane (const Plane* plane, const Point* point)
{
    return dot_Point (&plane->normal, point) + plane->offset;
}


void proj_Plane (Point* dst, const Point* u, const Plane* plane)
{
    Point tmp;
    proj_Point (&tmp, u, &plane->normal);
    diff_Point (dst, u, &tmp);
}


void barycentric_Plane (Plane* dst, const Plane* plane, const Point* point)
{
        /* NOTE: The given point must be relative to the plane!
         * Obtain this by subtracting any point on the plane.
         */
#if 1
    real inv;
    inv = 1 / dot_Point (&plane->normal, point);
    dst->offset = plane->offset * inv;
    scale_Point (&dst->normal, &plane->normal, inv);
#else
    real mag;
    mag = dot_Point (&plane->normal, point);
    dst->offset = plane->offset / mag;
    scale_Point (&dst->normal, &plane->normal, 1 / mag);
#endif
}

    void
barycentric_Point (Point* bpoint, const Point* isect,
                   const BarySimplex* simplex)
{
    uint i;
    bpoint->coords[0] = 1;
    UFor( i, NDimensions-1 )
    {
        bpoint->coords[i+1] =
            dist_Plane (&simplex->barys[i], isect);
        bpoint->coords[0] -= bpoint->coords[i+1];
    }
}

    real
area_Simplex (const Simplex* simplex)
{
    PointXfrm A;
    real x;
    {:for (i ; NDims-1)
        diff_Point (&A.pts[i],
                    &simplex->pts[i+1],
                    &simplex->pts[0]);
    }
    zero_Point (&A.pts[NDims-1]);

    transpose_PointXfrm (&A, &A);
    trxfrm_PointXfrm (&A, &A, &A);
    A.pts[NDims-1].coords[NDims-1] = 1;
    x = det_PointXfrm (&A);
    x = .5 * sqrt (x);
    return x;
}

    bool
degenerate_Simplex (const Simplex* raw)
{
    BarySimplex tmp;
    return !init_BarySimplex (&tmp, raw);
}

    bool
init_BarySimplex (BarySimplex* elem, const Simplex* raw)
{
    uint i;
    PointXfrm surf;
    Plane* plane;
    bool good = true;

    UFor( i, NDimensions-1 )
        diff_Point (&surf.pts[1+i], &raw->pts[1+i], &raw->pts[0]);

    plane = &elem->plane;
    row_minors_PointXfrm (&plane->normal, &surf, 0);
    checker_negate_Point (&plane->normal);
    init_Plane (plane, &plane->normal, &raw->pts[0]);
    copy_Point (&surf.pts[0], &plane->normal);

    UFor( i, NDimensions-1 )
    {
        if (!approx_eql (0, dot_Point (&surf.pts[0], &surf.pts[i+1]),
                         magnitude_Point (&surf.pts[i+1]), 1e1))
            good = false;

        plane = &elem->barys[i];
        row_minors_PointXfrm (&plane->normal, &surf, 1+i);
        checker_negate_Point (&plane->normal);
        init_Plane (plane, &plane->normal, &raw->pts[0]);
        barycentric_Plane (plane, plane, &surf.pts[1+i]);

        if (!approx_eql (1, dist_Plane (plane, &raw->pts[1+i]),
                         plane->offset, 5e2))
            good = false;
    }
    return good;
}

    bool
hit_Plane (real* restrict ret_dist,
           const Point* restrict origin,
           const Point* restrict direct,
           const Plane* restrict plane)
{
    real dist, dot;

    dist = dist_Plane (plane, origin);
    dot = dot_Point (&plane->normal, direct);

    if (dot < 0)
    {
        if (dist < 0)  return false;
        dot = - dot;
    }
    else if (dot > 0)
    {
        if (dist > 0)  return false;
        dist = - dist;
    }
    else
    {
        return false;
    }

    *ret_dist = (1/dot) * dist;
    return true;
}

    /** This is the simplest BarySimplex intersection test to read.**/
static inline
    bool
isect_BarySimplex (real* restrict ret_dist,
                   const Ray* restrict ray,
                   const BarySimplex* restrict elem,
                   Trit front)
{
    const real fuzz = -128 * Epsilon_real;
    real dist, dot, bcoord_sum;
    BaryPoint bpoint;
    Point isect;

    dist = dist_Plane (&elem->plane, &ray->origin);
    dot = dot_Point (&elem->plane.normal, &ray->direct);

    if (dot < 0 && front != Nil)
    {
        if (dist < 0)  return false;
        dot = - dot;
    }
    else if (dot > 0 && front != Yes)
    {
        if (dist > 0)  return false;
        dist = - dist;
    }
    else
    {
        return false;
    }

    bcoord_sum = max_uint(dist,1)*fuzz;
    dist *= 1 / dot;
    follow_Ray (&isect, ray, dist);

    {:for (i ; NDims-1)
        bpoint.coords[i] = dist_Plane (&elem->barys[i], &isect);
        if (bpoint.coords[i] < fuzz)  return false;
        bcoord_sum += bpoint.coords[i];
        if (bcoord_sum > 1)  return false;
    }
    *ret_dist = dist;
    return true;
}

static inline
    bool
delayed_div_isect_BarySimplex (real* restrict ret_dist,
                               const Ray* restrict ray,
                               const BarySimplex* restrict elem,
                               Trit front)
{
    const real fuzz = -128 * Epsilon_real;
    real dist, dot, bcoord_sum;
    BaryPoint bpoint;
    Point isect;

    dist = dist_Plane (&elem->plane, &ray->origin);
    dot = dot_Point (&elem->plane.normal, &ray->direct);

    if (dot < 0 && front != Nil)
    {
        if (dist < 0)  return false;
        dot = - dot;
    }
    else if (dot > 0 && front != Yes)
    {
        if (dist > 0)  return false;
        dist = - dist;
    }
    else
    {
        return false;
    }

    Op_Point_21010( &isect
                    ,+, dist*, &ray->direct
                    ,   dot*, &ray->origin );

    bcoord_sum = dist*fuzz;
    {:for (i ; NDims-1)
        bpoint.coords[i] =
            dot_Point (&elem->barys[i].normal, &isect)
            + elem->barys[i].offset * dot;
        if (bpoint.coords[i] < fuzz)  return false;
        bcoord_sum += bpoint.coords[i];
        if (bcoord_sum > dot)  return false;
    }

    dot = 1 / dot;
    *ret_dist = dist * dot;
    return true;
}

static inline
    bool
superfast_isect_BarySimplex (real* restrict ret_dist,
                             const Ray* restrict ray,
                             const BarySimplex* restrict elem,
                             Trit front)
{
    const real fuzz = -128 * Epsilon_real;
    real dist, dot, bcoord_sum;
    BaryPoint bpoint;
    Point isect;

    dist = dist_Plane (&elem->plane, &ray->origin);
    bcoord_sum = dist*fuzz;
    dot = - dot_Point (&elem->plane.normal, &ray->direct);
    if (dot > 0 && front != Nil)
    {
        if (dist < 0)  return false;

        Op_Point_21010( &isect
                        ,+, dist*, &ray->direct
                        ,   dot*, &ray->origin );

        {:for (i ; NDims-1)
            bpoint.coords[i] =
                dot_Point (&elem->barys[i].normal, &isect)
                + elem->barys[i].offset * dot;
            if (bpoint.coords[i] < fuzz)  return false;
        }

        {:for (i ; NDims-1)
            bcoord_sum += bpoint.coords[i];
        }

        if (bcoord_sum > dot)  return false;
    }
    else if (dot < 0 && front != Yes)
    {
        if (dist > 0)  return false;

        Op_Point_21010( &isect
                        ,+, dist*, &ray->direct
                        ,   dot*, &ray->origin );

        {:for (i ; NDims-1)
            bpoint.coords[i] =
                dot_Point (&elem->barys[i].normal, &isect)
                + elem->barys[i].offset * dot;
            if (bpoint.coords[i] > - fuzz)  return false;
        }

        {:for (i ; NDims-1)
            bcoord_sum += bpoint.coords[i];
        }

        if (bcoord_sum < dot)  return false;
    }
    else
    {
        return false;
    }

        /* I see no time penalty for this division.*/
        /* *ret_dist = dist / dot; */
    dot = 1 / dot;
    *ret_dist = dist * dot;
    return true;
}

    bool
hit_BarySimplex (real* restrict ret_dist,
                 const Ray* restrict ray,
                 const BarySimplex* restrict elem,
                 Trit front)
{
    return
#if 1
        isect_BarySimplex
#elif 0
        delayed_div_isect_BarySimplex
#else
        superfast_isect_BarySimplex
#endif
        (ret_dist, ray, elem, front);
}

