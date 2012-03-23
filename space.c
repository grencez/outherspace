
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

#endif  /* #ifndef __OPENCL_VERSION__ */


    /* a - b */
void diff_Point (Point* dst, const Point* a, const Point* b)
{
    Op_Point_200( dst, -, a, b );
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
    Op_Point_200( dst ,+, a , b );
}

    void
prod_Point (Point* dst, const Point* a, const Point* b)
{
    Op_Point_200( dst ,*, a , b );
}

    void
quot_Point (Point* dst, const Point* a, const Point* b)
{
    Op_Point_200( dst ,/, a , b );
}

void scale_Point (Point* dst, const Point* a, real k)
{
    Op_Point_10( dst ,k*, a );
}

    void
reci_Point (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (src->coords[i] == 0)  dst->coords[i] = 0;
        else                      dst->coords[i] = 1 / src->coords[i];
    }
}

    void
quot1_Point (Point* dst, const Point* src, real x)
{
    Op_20s( real, NDimensions, dst->coords ,/, src->coords , x );
}

    void
follow_Ray (Point* isect, const Ray* ray, real mag)
{
    Op_2020s( real, NDims, isect->coords
              ,+, ray->origin.coords
              ,   *, ray->direct.coords
              ,      mag );
}

void zero_Point (Point* a)
{
    uint i;
    UFor( i, NDimensions )
        a->coords[i] = 0;
}

    bool
equal_Point (const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
        if (a->coords[i] != b->coords[i])
            return false;
    return true;
}

    bool
ordered_Point (const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (a->coords[i] < b->coords[i])  return true;
        if (a->coords[i] > b->coords[i])  return false;
    }
    return true;
}

void copy_Point (Point* dst, const Point* src)
{
    *dst = *src;
}

void negate_Point (Point* dst, const Point* src)
{
    Op_Point_10( dst ,-, src );
}

    void
checker_negate_Point (Point* p)
{
    uint i;
    UFor( i, NDimensions / 2 )
        p->coords[2*i+1] = - p->coords[2*i+1];
}

    real
magnitude_Point (const Point* a)
{
    return sqrt (dot_Point (a, a));
}

    real
dist_Point (const Point* a, const Point* b)
{
    Point c;
    diff_Point (&c, a, b);
    return magnitude_Point (&c);
}

void normalize_Point (Point* dst, const Point* a)
{
    scale_Point (dst, a, 1 / magnitude_Point (a));
}

void proj_Point (Point* dst, const Point* a, const Point* b)
{
    scale_Point (dst, b, dot_Point (a, b) / dot_Point (b, b));
}

    void
orth_Point (Point* dst, const Point* a, const Point* b)
{
    Point tmp;
    proj_Point (&tmp, a, b);
    diff_Point (dst, a, &tmp);
}

    void
proj_unit_Point (Point* dst, const Point* a, const Point* b)
{
    scale_Point (dst, b, dot_Point (a, b));
}

    void
orth_unit_Point (Point* dst, const Point* a, const Point* b)
{
    Point tmp;
    proj_unit_Point (&tmp, a, b);
    diff_Point (dst, a, &tmp);
}

    /* /dot/ is the dot product of /p/ and /normal/ */
    void
reflect_Point (Point* refl, const Point* p,
               const Point* normal, real dot)
{
    real d2;
    d2 = 2 * dot;
    Op_Point_2100( refl
                   ,-, d2*, normal
                   ,   p );
}


