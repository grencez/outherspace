
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
    PointXfrm surf;
    Plane* plane;

    diff_Point (&surf.pts[1], &raw->pts[1], &raw->pts[0]);
    diff_Point (&surf.pts[2], &raw->pts[2], &raw->pts[0]);

    plane = &elem->plane;

    row_minors_PointXfrm (&plane->normal, &surf, 0);
    plane->normal.coords[1] = - plane->normal.coords[1];
    init_Plane (plane, &plane->normal, &raw->pts[0]);

    AssertEqual_real( 0, dot_Point (&plane->normal, &surf.pts[1]) );
    AssertEqual_real( 0, dot_Point (&plane->normal, &surf.pts[2]) );

    copy_Point (&surf.pts[0], &plane->normal);

    plane = &elem->barys[0];

    row_minors_PointXfrm (&plane->normal, &surf, 1);
    plane->normal.coords[1] = - plane->normal.coords[1];
    init_Plane (plane, &plane->normal, &raw->pts[0]);
    barycentric_Plane (plane, plane, &raw->pts[1]);

    plane = &elem->barys[1];

    row_minors_PointXfrm (&plane->normal, &surf, 2);
    plane->normal.coords[1] = - plane->normal.coords[1];
    init_Plane (plane, &plane->normal, &raw->pts[0]);
    barycentric_Plane (plane, plane, &raw->pts[2]);
}


bool hit_BarySimplex (real* restrict ret_dist,
                      const Point* restrict origin,
                      const Point* restrict dir,
                      const BarySimplex* restrict elem)
{
    real dist, u, v;
    Point isect;

    dist = (distance_Plane (&elem->plane, origin)
            / - dot_Point (&elem->plane.normal, dir));
    if (dist < 0)  return false;
    
    scale_Point (&isect, dir, dist);
    summ_Point (&isect, &isect, origin);
    
    u = distance_Plane (&elem->barys[0], &isect);
    if (u < 0 || u > 1)  return false;
    v = distance_Plane (&elem->barys[1], &isect);
    if (v < 0 || u + v > 1)  return false;

    *ret_dist = dist;
    return true;
}

