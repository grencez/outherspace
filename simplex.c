
#ifndef __OPENCL_VERSION__
#include "simplex.h"

#include <assert.h>
#include <math.h>

void output_Simplex (FILE* out, const Simplex* elem)
{
    uint pi;
    const char* delim = "";
    fputc ('[', out);
    UFor( pi, NDimensions )
    {
        fputs (delim, out);
        output_Point (out, &elem->pts[pi]);
        delim = " ";
    }
    fputc (']', out);
}
#endif  /* #ifndef __OPENCL_VERSION__ */

#if NDimensions != 4
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
             const Point* restrict origin,
             const Point* restrict dir,
             const Simplex* restrict elem)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point edge1, edge2, tvec, pvec, qvec;
    real det, inv_det;
    real u, v;

#if 0
    diff_Point (&tvec, origin, &elem->pts[0]);
    diff_Point (&edge2, &elem->pts[2], &elem->pts[0]);
    diff_Point (&edge1, &elem->pts[1], &elem->pts[0]);
#else
    Op_Point_200( &tvec  ,-, origin        , &elem->pts[0] );
    Op_Point_200( &edge2 ,-, &elem->pts[2] , &elem->pts[0] );
    Op_Point_200( &edge1 ,-, &elem->pts[1] , &elem->pts[0] );
#endif
    cross_Point (&pvec, dir, &edge2);
    u = dot_Point (&tvec, &pvec);
    det = dot_Point (&edge1, &pvec);

    if (det > epsilon)
    {
        if (u < 0 || u > det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (dir, &qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
        if (u > 0 || u < det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (dir, &qvec);
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

#else  /*V NDimensions == 4 V*/

static
    void
solve_3x3 (real x[3], const PointXfrm* A, const real b[3])
{
    uint i;
    real det, invdet;

    det = det3_PointXfrm (A, 0, 1, 2, 0, 1, 2);
    if (-Epsilon_real < det && det < Epsilon_real)
    {
        x[0] = -1;
            /* puts ("A"); */
        return;
    }

    invdet = 1/det;
    UFor( i, 3 )
    {
        uint j;
        PointXfrm S;
        copy_PointXfrm (&S, A);
        UFor( j, 3 )
            S.pts[j].coords[i] = b[j];
        x[i] = invdet * det3_PointXfrm (&S, 0, 1, 2, 0, 1, 2);
    }
}

    /* BROKE! */
    bool
hit_Simplex (real* restrict ret_dist,
             const Point* restrict origin,
             const Point* restrict direct,
             const Simplex* restrict elem)
{
    uint i;
    uint axes[3];
    real b[3];
    real rayt;
    real offset;
    real x[3];
    PointXfrm A, B;
    Point normal, isect;
        /* puts ("Z"); */

    diff_Point (&B.pts[1], &elem->pts[1], &elem->pts[0]);
    diff_Point (&B.pts[2], &elem->pts[2], &elem->pts[0]);
    diff_Point (&B.pts[3], &elem->pts[3], &elem->pts[0]);

    row_minors_PointXfrm (&normal, &B, 0);
    checker_negate_Point (&normal);

    {
        uint maxidx = 0;
        real maxcoord;
        maxcoord = fabs (normal.coords[maxidx]);
        UFor( i, NDimensions-1 )
        {
            real a;
            a = fabs (normal.coords[i+1]);
            if (a > maxcoord)
            {
                maxidx = i=1;
                maxcoord = a;
            }
        }

        UFor( i, NDimensions )
        {
            if (i < maxidx)  axes[i] = i;
            if (i > maxidx)  axes[i-1] = i;
        }
        normal.coords[maxidx] = 0;
    }

    normalize_Point (&normal, &normal);

    rayt = dot_Point (&normal, direct);
    if (-Epsilon_real < rayt && rayt < Epsilon_real)
    {
            /* puts ("B"); */
        return false;
    }

    
    offset = - dot_Point (&elem->pts[0], &normal);
    rayt = - (offset + dot_Point (&normal, origin)) / rayt;
    if (rayt < 0)
    {
            /* puts ("E"); */
        return false;
    }

        /* Calculate the intersection point of
         * the ray and embedding hyperplane.
         */
    scale_Point (&isect, direct, rayt);
    summ_Point (&isect, &isect, origin);

        /* Calculate the equation result values. Note that the dominant
         * axes are precomputed and stored in tet.axes.
         */
    UFor( i, 3 )
    {
        b[i] = isect.coords[axes[i]] - elem->pts[0].coords[axes[i]];
    }

    
    zero_PointXfrm (&A);
    UFor( i, 3 )
    {
        uint j;
        UFor( j, 3 )
            A.pts[i].coords[j] = B.pts[i+1].coords[axes[j]];
    }


        /* Solve the system of three equations and three unknowns.*/
    solve_3x3 (x, &A, b);

    if (x[0] < 0 || x[1] < 0 || x[2] < 0 || (x[0]+x[1]+x[2] > 1))
    {
            /* puts ("C"); */
        return false;
    }
    *ret_dist = rayt;
    return true;
}
#endif


static void cross3 (real dst[3], const real a[3], const real b[3])
{
    dst[0] = a[1] * b[2] - a[2] * b[1];
    dst[1] = a[2] * b[0] - a[0] * b[2];
    dst[2] = a[0] * b[1] - a[1] * b[0];
}


static real dot3 (const real a[3], const real b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool hit_proj_Simplex (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Simplex* restrict elem,
                        const PointXfrm* restrict view_basis)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point tmp_diff, tmp_proj;
    real dir[3], edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    real det, inv_det;
    PointXfrm ray_basis;
    real u, v;

    copy_PointXfrm (&ray_basis, view_basis);
    copy_Point (&ray_basis.pts[2], kd_dir);

    trxfrm_Point (&tmp_proj, &ray_basis, kd_dir);
    dir[0] = tmp_proj.coords[0];
    dir[1] = tmp_proj.coords[1];
    dir[2] = tmp_proj.coords[2];

    diff_Point (&tmp_diff, &elem->pts[1], &elem->pts[0]);
    trxfrm_Point (&tmp_proj, &ray_basis, &tmp_diff);
    edge1[0] = tmp_proj.coords[0];
    edge1[1] = tmp_proj.coords[1];
    edge1[2] = tmp_proj.coords[2];

    diff_Point (&tmp_diff, &elem->pts[2], &elem->pts[0]);
    trxfrm_Point (&tmp_proj, &ray_basis, &tmp_diff);
    edge2[0] = tmp_proj.coords[0];
    edge2[1] = tmp_proj.coords[1];
    edge2[2] = tmp_proj.coords[2];

    diff_Point (&tmp_diff, origin, &elem->pts[0]);
    trxfrm_Point (&tmp_proj, &ray_basis, &tmp_diff);
    tvec[0] = tmp_proj.coords[0];
    tvec[1] = tmp_proj.coords[1];
    tvec[2] = tmp_proj.coords[2];

    cross3 (pvec, dir, edge2);

    u = dot3 (tvec, pvec);
    det = dot3 (edge1, pvec);

    if (det > epsilon)
    {
        if (u < 0 || u > det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
        if (u > 0 || u < det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v > 0 || u + v < det)
            return false;
    }
    else
    {
        return false;
    }

    inv_det = 1 / det;
    *dist = dot3 (edge2, qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return *dist >= 0;
}

bool hit_weak_Simplex (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Simplex* restrict elem)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point kd_edge1, kd_edge2, kd_tvec;
    real dir[3], edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    real det, inv_det;
    real u, v;

    diff_Point (&kd_edge1, &elem->pts[1], &elem->pts[0]);
    diff_Point (&kd_edge2, &elem->pts[2], &elem->pts[0]);
    diff_Point (&kd_tvec,  origin,        &elem->pts[0]);

    dir[0] = - dot_Point (kd_dir, kd_dir);
    dir[1] =   dot_Point (kd_dir, &kd_edge1);
    dir[2] =   dot_Point (kd_dir, &kd_edge2);

    edge1[0] = - dir[1];
    edge1[1] =   dot_Point (&kd_edge1, &kd_edge1);
    edge1[2] =   dot_Point (&kd_edge1, &kd_edge2);

    edge2[0] = - dir[2];
    edge2[1] =   edge1[2];
    edge2[2] =   dot_Point (&kd_edge2, &kd_edge2);

    tvec[0] = - dot_Point (kd_dir,    &kd_tvec);
    tvec[1] =   dot_Point (&kd_edge1, &kd_tvec);
    tvec[2] =   dot_Point (&kd_edge2, &kd_tvec);

    cross3 (pvec, dir, edge2);

    u = dot3 (tvec, pvec);
    det = dot3 (edge1, pvec);

    if (det > epsilon)
    {
        if (u < 0 || u > det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
        if (u > 0 || u < det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v > 0 || u + v < det)
            return false;
    }
    else
    {
        return false;
    }

    inv_det = 1 / det;
    *dist = dot3 (edge2, qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return *dist >= 0;
}


void init_Plane (Plane* plane, const Point* normal, const Point* point)
{
    normalize_Point (&plane->normal, normal);
    plane->offset = - dot_Point (&plane->normal, point);
}


real distance_Plane (const Plane* plane, const Point* point)
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
            distance_Plane (&simplex->barys[i], isect);
        bpoint->coords[0] -= bpoint->coords[i+1];
    }
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

        if (!approx_eql (1, distance_Plane (plane, &raw->pts[1+i]),
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

    dist = distance_Plane (plane, origin);
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

    bool
hit_BarySimplex (real* restrict ret_dist,
                 const Point* restrict origin,
                 const Point* restrict dir,
                 const BarySimplex* restrict elem)
{
#if 0
    uint i;
    real dist, dot, bcoord_sum;
    BaryPoint bpoint;
    Point isect;

    dist = distance_Plane (&elem->plane, origin);
    dot = dot_Point (&elem->plane.normal, dir);

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

# if 0
    dist *= 1 / dot;
    Op_Point_2010 ( &isect ,+, origin ,dist*, dir );
    
    bcoord_sum = 0;
    UFor( i, NDimensions-1 )
    {
        bpoint.coords[i] = distance_Plane (&elem->barys[i], &isect);
        if (bpoint.coords[i] < 0)  return false;
        bcoord_sum += bpoint.coords[i];
        if (bcoord_sum > 1)  return false;
    }
    *ret_dist = dist;
# else

    
    Op_Point_21010( &isect
                    ,+, dist*, dir
                    ,   dot*, origin );
    
    bcoord_sum = 0;
    UFor( i, NDimensions-1 )
    {
        bpoint.coords[i] =
            dot_Point (&elem->barys[i].normal, &isect)
            + elem->barys[i].offset * dot;
        if (bpoint.coords[i] < 0)  return false;
        bcoord_sum += bpoint.coords[i];
        if (bcoord_sum > dot)  return false;
    }

    dot = 1 / dot;
    *ret_dist = dist * dot;
# endif

    return true;
#else
    uint i;
    real dist, dot, bcoord_sum = 0;
    BaryPoint bpoint;
    Point isect;

    dist = distance_Plane (&elem->plane, origin);
    dot = - dot_Point (&elem->plane.normal, dir);
    if (dot > 0)
    {
        if (dist < 0)  return false;

        Op_Point_21010( &isect
                        ,+, dist*, dir
                        ,   dot*, origin );

        UFor( i, NDimensions-1 )
        {
            bpoint.coords[i] =
                dot_Point (&elem->barys[i].normal, &isect)
                + elem->barys[i].offset * dot;
            if (bpoint.coords[i] < 0)  return false;
        }

        UFor( i, NDimensions-1 )
            bcoord_sum += bpoint.coords[i];

        if (bcoord_sum > dot)  return false;
    }
    else if (dot < 0)
    {
        if (dist > 0)  return false;

        Op_Point_21010( &isect
                        ,+, dist*, dir
                        ,   dot*, origin );

        UFor( i, NDimensions-1 )
        {
            bpoint.coords[i] =
                dot_Point (&elem->barys[i].normal, &isect)
                + elem->barys[i].offset * dot;
            if (bpoint.coords[i] > 0)  return false;
        }

        UFor( i, NDimensions-1 )
            bcoord_sum += bpoint.coords[i];

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
#endif
}

