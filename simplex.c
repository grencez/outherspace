
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
    real inv;
    inv = 1 / distance_Plane (plane, point);
    dst->offset = plane->offset * inv;
    scale_Point (&dst->normal, &plane->normal, inv);

    AssertEqual_real( 1, distance_Plane (dst, point) );
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
        AssertEqual_real( 0, dot_Point (&surf.pts[0], &surf.pts[i+1]) );
        plane = &elem->barys[i];
        row_minors_PointXfrm (&plane->normal, &surf, 1+i);
        plane->normal.coords[1] = - plane->normal.coords[1];
        init_Plane (plane, &plane->normal, &raw->pts[0]);
        barycentric_Plane (plane, plane, &raw->pts[1+i]);
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

