
#include "space.h"

#include <math.h>

void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
            /* fprintf (out, "%s%4.1f", delim, point->coords[ci]); */
        fprintf (out, "%s%.1f", delim, point->coords[ci]);
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

void negate_Point (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = - src->coords[i];
}

void set_Point (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = src->coords[i];
}

real magnitude_Point (const Point* a)
{
    return sqrt (dot_Point (a, a));
}

void normalize_Point (Point* a)
{
    scale_Point (a, a, 1 / magnitude_Point (a));
}

bool facing_BoundingPlane (uint dim, real plane,
                           const Point* origin, const Point* dir)
{
    tristate sign;
    sign = compare_real (plane, origin->coords[dim]);
    sign *= signum_real (dir->coords[dim]);
    return sign >= 0;
}

bool hit_BoundingPlane (Point* entrance,
                        uint dim, real plane,
                        const BoundingBox* box,
                        const Point* origin, const Point* dir)
{
    uint i;
    bool didhit = true;
    real coeff;

    if (! facing_BoundingPlane (dim, plane, origin, dir))  return false;

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
            if (x < box->min_corner.coords[i] ||
                x > box->max_corner.coords[i])
                didhit = false;
        }
        entrance->coords[i] = x;
    }
    return didhit;
}

bool hit_outer_BoundingBox (Point* entrance,
                            const BoundingBox* box,
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
                cost.coords[dim] = Max_real;
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
                cost.coords[dim] = Max_real;
            else
                cost.coords[dim] = ((box->max_corner.coords[dim]
                                     - origin->coords[dim])
                                    / dir->coords[dim]);
        }
        else
        {
            cost.coords[dim] = Max_real;
        }
    }

    hit_cost = Max_real;
    hit_dim = 0;

    UFor( dim, NDimensions )
    {
        if (cost.coords[dim] < hit_cost)
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

bool inside_BoundingBox (const BoundingBox* box, const Point* point)
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

