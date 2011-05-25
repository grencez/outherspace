
#ifndef __OPENCL_VERSION__
#include "space.h"

#include <assert.h>
#include <math.h>
#include <string.h>

void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
            /* fprintf (out, "%s%4.1f", delim, point->coords[ci]); */
        fprintf (out, "%s%.2f", delim, point->coords[ci]);
            /* delim = ", "; */
        delim = " ";
    }
    fputc (')', out);
}

void output_BoundingBox (FILE* out, const BoundingBox* box)
{
    fputs ("BoundingBox: ", out);
    output_Point (out, &box->min_corner);
    fputs (" to ", out);
    output_Point (out, &box->max_corner);
}
#endif  /* #ifndef __OPENCL_VERSION__ */


    /* a - b */
void diff_Point (Point* dst, const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = a->coords[i] - b->coords[i];
}

real dot_Point (const Point* a, const Point* b)
{
    uint i;
    real sum = 0;
    UFor( i, NDimensions )
        sum += a->coords[i] * b->coords[i];
    return sum;
}

    /* a + b */
void summ_Point (Point* dst, const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = a->coords[i] + b->coords[i];
}

void scale_Point (Point* dst, const Point* a, real k)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = k * a->coords[i];
}

void zero_Point (Point* a)
{
    uint i;
    UFor( i, NDimensions )
        a->coords[i] = 0;
}

void copy_Point (Point* dst, const Point* src)
{
#ifdef __OPENCL_VERSION__
    *dst = *src;
#else
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = src->coords[i];
#endif
}

void copy_BoundingBox (BoundingBox* dst, const BoundingBox* src)
{
#ifdef __OPENCL_VERSION__
    *dst = *src;
#else
    uint i;
    UFor( i, NDimensions )
    {
        dst->min_corner.coords[i] = src->min_corner.coords[i];
        dst->max_corner.coords[i] = src->max_corner.coords[i];
    }
#endif
}

void negate_Point (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = - src->coords[i];
}

    void
checker_negate_Point (Point* p)
{
    uint i;
    UFor( i, NDimensions / 2 )
        p->coords[2*i+1] = - p->coords[2*i+1];
}

real magnitude_Point (const Point* a)
{
    return sqrt (dot_Point (a, a));
}

void normalize_Point (Point* dst, const Point* a)
{
    scale_Point (dst, a, 1 / magnitude_Point (a));
}

void proj_Point (Point* dst, const Point* a, const Point* b)
{
    scale_Point (dst, b, dot_Point (a, b) / dot_Point (b, b));
}

tristate facing_BoundingPlane (uint dim, real plane,
                               const Point* origin, const Point* dir)
{
    tristate sign;
    sign = compare_real (plane, origin->coords[dim]);
    return mul_signum (sign, signum_real (dir->coords[dim]));
}

    /* Assume ray is coming from inside BoundingBox.*/
bool hit_inner_BoundingPlane (Point* entrance,
                              uint dim, real plane,
                              __global const BoundingBox* box,
                              const Point* origin, const Point* dir)
{
    uint i;
    bool didhit = true;
    real coeff;
    tristate facing;

    facing = facing_BoundingPlane (dim, plane, origin, dir);

    if (facing < 0)  return false;

    if (facing == 0)
    {
            /* It's on plane.*/
        UFor( i, NDimensions )
        {
            if (i == dim)
            {
                entrance->coords[i] = origin->coords[i];
            }
            else if (origin->coords[i] < box->min_corner.coords[i])
            {
                if (dir->coords[i] <= 0)  didhit = false;
                else  entrance->coords[i] = box->min_corner.coords[i];
            }
            else if (origin->coords[i] > box->max_corner.coords[i])
            {
                if (dir->coords[i] >= 0)  didhit = false;
                else  entrance->coords[i] = box->max_corner.coords[i];
            }
            else
            {
                entrance->coords[i] = origin->coords[i];
            }
        }
        return didhit;
    }

    coeff = (plane - origin->coords[dim]) / dir->coords[dim];
    
    UFor( i, NDimensions )
    {
        real x;
        if (i == dim)
        {
            x = plane;
        }
        else
        {
            x = origin->coords[i] + coeff * dir->coords[i];
            if (x < box->min_corner.coords[i])
            {
                if (dir->coords[i] < 0)  didhit = false;
                else  x = box->min_corner.coords[i];
            }
            else if (x > box->max_corner.coords[i])
            {
                if (dir->coords[i] > 0)  didhit = false;
                else  x = box->max_corner.coords[i];
            }
        }
        entrance->coords[i] = x;
    }
    return didhit;
}

    /* Assume ray is coming from outside BoundingBox.*/
bool hit_outer_BoundingBox (Point* entrance,
                            __global const BoundingBox* box,
                            const Point* origin, const Point* dir)
{
    uint dim;
    Point cost;
    real hit_cost;
    uint hit_dim;

    UFor( dim, NDimensions )
    {
        if (dir->coords[dim] > 0)
        {
            if (origin->coords[dim] > box->max_corner.coords[dim])
                return false;
            if (origin->coords[dim] > box->min_corner.coords[dim])
                cost.coords[dim] = Small_real;
            else
                cost.coords[dim] = ((box->min_corner.coords[dim]
                                     - origin->coords[dim])
                                    / dir->coords[dim]);
        }
        else if (dir->coords[dim] < 0)
        {
            if (origin->coords[dim] < box->min_corner.coords[dim])
                return false;
            if (origin->coords[dim] < box->max_corner.coords[dim])
                cost.coords[dim] = Small_real;
            else
                cost.coords[dim] = ((box->max_corner.coords[dim]
                                     - origin->coords[dim])
                                    / dir->coords[dim]);
        }
        else
        {
            cost.coords[dim] = Small_real;
        }
    }

    hit_cost = 0;
    hit_dim = 0;

    UFor( dim, NDimensions )
    {
        if (cost.coords[dim] > hit_cost)
        {
            hit_cost = cost.coords[dim];
            hit_dim = dim;
        }
    }

    UFor( dim, NDimensions )
    {
        if (dim == hit_dim)
        {
            if (dir->coords[dim] > 0)
                entrance->coords[dim] = box->min_corner.coords[dim];
            else
                entrance->coords[dim] = box->max_corner.coords[dim];
        }
        else
        {
            entrance->coords[dim] = (origin->coords[dim]
                                     + hit_cost * dir->coords[dim]);
            if (dir->coords[dim] > 0)
            {
                if (entrance->coords[dim] > box->max_corner.coords[dim])
                    return false;
            }
            else
            {
                if (entrance->coords[dim] < box->min_corner.coords[dim])
                    return false;
            }
        }
    }

    return true;
}

bool hit_BoundingBox (Point* entrance,
                      const BoundingBox* box,
                      const Point* origin, const Point* dir)
{
    uint dim;
    Point cost;
    real hit_cost;

    UFor( dim, NDimensions )
    {
        if (dir->coords[dim] > 0)
        {
            real bound;
            if (origin->coords[dim] > box->max_corner.coords[dim])
                return false;
            if (origin->coords[dim] > box->min_corner.coords[dim])
                bound = box->max_corner.coords[dim];
            else
                bound = box->min_corner.coords[dim];

            cost.coords[dim] = ((bound - origin->coords[dim])
                                / dir->coords[dim]);
        }
        else if (dir->coords[dim] < 0)
        {
            real bound;
            if (origin->coords[dim] < box->min_corner.coords[dim])
                return false;
            if (origin->coords[dim] < box->max_corner.coords[dim])
                bound = box->min_corner.coords[dim];
            else
                bound = box->max_corner.coords[dim];

            cost.coords[dim] = ((bound - origin->coords[dim])
                                / dir->coords[dim]);
        }
        else
        {
            cost.coords[dim] = Max_real;
        }
    }

    hit_cost = Max_real;

    UFor( dim, NDimensions )
        if (cost.coords[dim] < hit_cost)
            hit_cost = cost.coords[dim];

    UFor( dim, NDimensions )
        entrance->coords[dim] = (origin->coords[dim]
                                 + hit_cost * dir->coords[dim]);
    return true;
}

void init_BoundingBox (BoundingBox* box, uint npoints, const Point* points)
{
    uint i;

    if (npoints == 0)
    {
        zero_Point (&box->min_corner);
        zero_Point (&box->max_corner);
    }
    else
    {
        copy_Point (&box->min_corner, &points[0]);
        copy_Point (&box->max_corner, &points[0]);
    }

    UFor( i, npoints )
        adjust_BoundingBox (box, &points[i]);
}

void adjust_BoundingBox (BoundingBox* box, const Point* point)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (point->coords[i] < box->min_corner.coords[i])
            box->min_corner.coords[i] = point->coords[i];
        else if (point->coords[i] > box->max_corner.coords[i])
            box->max_corner.coords[i] = point->coords[i];
    }
}

    void
centroid_BoundingBox (Point* dst, const BoundingBox* box)
{
    summ_Point (dst, &box->min_corner, &box->max_corner);
    scale_Point (dst, dst, .5);
}

bool inside_BoundingBox (__global const BoundingBox* box, const Point* point)
{
    bool inside = true;
    uint i;
    UFor( i, NDimensions )
    {
        if (!(box->min_corner.coords[i] <= point->coords[i] &&
              box->max_corner.coords[i] >= point->coords[i]))
            inside = false;
    }
    return inside;
}

real surface_area_BoundingBox (const BoundingBox* box)
{
    Point delta;
    real sum = 0;
    uint i;

    diff_Point (&delta, &box->max_corner, &box->min_corner);

    UFor( i, NDimensions )
    {
        real prod = 1;
        uint j;
        UFor( j, NDimensions )
        {
            if (i != j)
            {
                prod *= delta.coords[j];
            }
        }
        sum += prod;
    }
    return 2 * sum;
}

void split_BoundingBox (BoundingBox* lo_box, BoundingBox* hi_box,
                        const BoundingBox* box,
                        uint split_dim, real split_pos)
{
    assert (split_pos < box->max_corner.coords[split_dim]);
    assert (split_pos > box->min_corner.coords[split_dim]);

    copy_BoundingBox (lo_box, box);
    copy_BoundingBox (hi_box, box);

    lo_box->max_corner.coords[split_dim] = split_pos;
    hi_box->min_corner.coords[split_dim] = split_pos;
}

