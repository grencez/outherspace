
#ifndef Point
#ifndef Point_H_
#define Point_H_
#define Point Point
#endif
#endif

#ifdef Point
#ifndef __OPENCL_VERSION__
#include "op.h"
#include <math.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

#define def(name)  ConcatifyDef(name##_,Point)
#define qualify qual_inline

qualify
    void
def(set) (Point* a, real x)
{
    Op_s( real, NDims, a->coords , x );
}

qualify
    void
def(copy) (Point* dst, const Point* src)
{
    *dst = *src;
}


qualify
    void
def(zero) (Point* a)
{
    def(set) (a, 0);
}

    /* a - b */
qualify
    void
def(diff) (Point* dst, const Point* a, const Point* b)
{
    Op_200( real, NDims, dst->coords ,-, a->coords, b->coords );
}

    /* a + b */
qualify
    void
def(summ) (Point* dst, const Point* a, const Point* b)
{
    Op_200( real, NDims, dst->coords ,+, a->coords , b->coords );
}

qualify
    void
def(prod) (Point* dst, const Point* a, const Point* b)
{
    Op_200( real, NDims, dst->coords ,*, a->coords , b->coords );
}

qualify
    void
def(quot) (Point* dst, const Point* a, const Point* b)
{
    Op_200( real, NDims, dst->coords ,/, a->coords , b->coords );
}

qualify
    real
def(dot) (const Point* a, const Point* b)
{
    uint i;
    real sum = 0;
    UFor( i, NDims )
        sum += a->coords[i] * b->coords[i];
    return sum;
}

qualify
    void
def(scale) (Point* dst, const Point* a, real k)
{
    Op_20s( real, NDims, dst->coords ,*, a->coords , k );
}

qualify
    void
def(reci) (Point* dst, const Point* src)
{
    uint i;
    UFor( i, NDims )
    {
        if (src->coords[i] == 0)  dst->coords[i] = 0;
        else                      dst->coords[i] = 1 / src->coords[i];
    }
}

qualify
    void
def(quot1) (Point* dst, const Point* src, real x)
{
    Op_20s( real, NDims, dst->coords ,/, src->coords , x );
}

qualify
    void
def(mix) (Point* dst, const Point* x, const Point* y, real a)
{
    const real* const xs = x->coords;
    const real* const ys = y->coords;
    {:for (i ; NDims)
        dst->coords[i] = xs[i] + (ys[i] - xs[i]) * a;
    }
}

    /** /dst = origin + direct * mag/ **/
qualify
    void
def(follow) (Point* dst, const Point* origin, const Point* direct, real mag)
{
    Op_2020s( real, NDims, dst->coords
              ,+, origin->coords
              ,   *, direct->coords
              ,      mag );
}

qualify
    bool
def(equal) (const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDims )
        if (a->coords[i] != b->coords[i])
            return false;
    return true;
}

qualify
    bool
def(ordered) (const Point* a, const Point* b)
{
    uint i;
    UFor( i, NDims )
    {
        if (a->coords[i] < b->coords[i])  return true;
        if (a->coords[i] > b->coords[i])  return false;
    }
    return true;
}

qualify
    void
def(negate) (Point* dst, const Point* src)
{
    Op_10( real, NDims, dst->coords ,-, src->coords );
}

qualify
    void
def(checker_negate) (Point* p)
{
    uint i;
    UFor( i, NDims / 2 )
        p->coords[2*i+1] = - p->coords[2*i+1];
}

qualify
    real
def(mag2) (const Point* a)
{
    return def(dot) (a, a);
}

qualify
    real
def(magnitude) (const Point* a)
{
    return sqrt (def(mag2) (a));
}

    /** L1 norm, also known as Manhattan norm or taxicab norm.**/
qualify
    real
def(taximag) (const Point* a)
{
    real sum = 0;
    {:for (i ; NDims)
        sum += fabs (a->coords[i]);
    }
    return sum;
}

    /** Infinity norm.**/
qualify
    real
def(maxmag) (const Point* a)
{
    real x = 0;
    {:for (i ; NDims)
        real y = fabs (a->coords[i]);
        if (y > x)  x = y;
    }
    return x;
}

qualify
    real
def(dist) (const Point* a, const Point* b)
{
    Point c;
    def(diff) (&c, a, b);
    return def(magnitude) (&c);
}

qualify
    real
def(dmag2) (const Point* a, const Point* b)
{
    Point c;
    def(diff) (&c, a, b);
    return def(mag2) (&c);
}

qualify
    void
def(normalize) (Point* dst, const Point* a)
{
    def(scale) (dst, a, 1 / def(magnitude) (a));
}

qualify
    void
def(proj) (Point* dst, const Point* a, const Point* b)
{
    def(scale) (dst, b, def(dot) (a, b) / def(dot) (b, b));
}

qualify
    void
def(orth) (Point* dst, const Point* a, const Point* b)
{
    Point tmp;
    def(proj) (&tmp, a, b);
    def(diff) (dst, a, &tmp);
}

qualify
    void
def(proj_unit) (Point* dst, const Point* a, const Point* b)
{
    def(scale) (dst, b, def(dot) (a, b));
}

qualify
    void
def(orth_unit) (Point* dst, const Point* a, const Point* b)
{
    Point tmp;
    def(proj_unit) (&tmp, a, b);
    def(diff) (dst, a, &tmp);
}

    /* /dot/ is the dot product of /p/ and /normal/ */
qualify
    void
def(reflect) (Point* refl, const Point* p,
              const Point* normal, real dot)
{
    real d2 = 2 * dot;
    Op_2100( real, NDims, refl->coords
             ,-, d2*, normal->coords
             ,   p->coords );
}

#undef qualify
#undef def
#undef Point
#endif

