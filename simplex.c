
#ifndef __OPENCL_VERSION__
#include "simplex.h"

#include <assert.h>
#include <math.h>

void output_Triangle (FILE* out, const Triangle* elem)
{
    uint pi;
    const char* delim = "";
    fputc ('[', out);
    UFor( pi, NTrianglePoints )
    {
        fputs (delim, out);
        output_Point (out, &elem->pts[pi]);
        delim = " ";
    }
    fputc (']', out);
}
#endif  /* #ifndef __OPENCL_VERSION__ */

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


bool hit_Triangle (real* restrict ret_dist,
                   const Point* restrict origin,
                   const Point* restrict dir,
                   const Triangle* restrict elem)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point edge1, edge2, tvec, pvec, qvec;
    real det, inv_det;
    real u, v;

    diff_Point (&tvec, origin, &elem->pts[0]);

    diff_Point (&edge2, &elem->pts[2], &elem->pts[0]);
    cross_Point (&pvec, dir, &edge2);

    diff_Point (&edge1, &elem->pts[1], &elem->pts[0]);
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

bool hit_proj_Triangle (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Triangle* restrict elem,
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

bool hit_weak_Triangle (real* restrict dist,
                        const Point* restrict origin,
                        const Point* restrict kd_dir,
                        const Triangle* restrict elem)
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


void init_BarySimplex (BarySimplex* elem, const PointXfrm* raw)
{
    uint i;
    PointXfrm surf;
    Plane* plane;

    UFor( i, NDimensions-1 )
        diff_Point (&surf.pts[1+i], &raw->pts[1+i], &raw->pts[0]);

    plane = &elem->plane;
    row_minors_PointXfrm (&plane->normal, &surf, 0);
    plane->normal.coords[1] = - plane->normal.coords[1];
    init_Plane (plane, &plane->normal, &raw->pts[0]);
    copy_Point (&surf.pts[0], &plane->normal);

    UFor( i, NDimensions-1 )
    {
        AssertApprox( 0, dot_Point (&surf.pts[0], &surf.pts[i+1]),
                      magnitude_Point (&surf.pts[i+1]), 1e1 );

        plane = &elem->barys[i];
        row_minors_PointXfrm (&plane->normal, &surf, 1+i);
        plane->normal.coords[1] = - plane->normal.coords[1];
        init_Plane (plane, &plane->normal, &raw->pts[0]);
        barycentric_Plane (plane, plane, &surf.pts[1+i]);

        AssertApprox( 1, distance_Plane (plane, &raw->pts[1+i]),
                      plane->offset, 5e2 );
    }
}


bool hit_BarySimplex (real* restrict ret_dist,
                      const Point* restrict origin,
                      const Point* restrict dir,
                      const BarySimplex* restrict elem)
{
    uint i;
    real dist, dot, bcoord_sum;
    real bcoords[NDimensions-1];
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
    
    dist *= 1 / dot;
    
    scale_Point (&isect, dir, dist);
    summ_Point (&isect, &isect, origin);
    
    bcoord_sum = 0;
    UFor( i, NDimensions-1 )
    {
        bcoords[i] = distance_Plane (&elem->barys[i], &isect);
        if (bcoords[i] < 0)  return false;
        bcoord_sum += bcoords[i];
        if (bcoord_sum > 1)  return false;
    }

    *ret_dist = dist;
    return true;
}

