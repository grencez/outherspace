
#include "op.h"
#include "pack.h"

#include <emmintrin.h>

    boolPack
pack_boolPack (const bool* src)
{
    union {
        boolPack u;
        uint v[realPackSz];
    } x;
    uint i;
    UFor( i, realPackSz )
        x.v[i] = src[i] ? Max_uint : 0;
    return x.u;
}

    void
scat_boolPack (bool* dst, boolPack src)
{
    int mask = _mm_movemask_ps (src);
    uint i;
    UFor( i, realPackSz )
        dst[i] = (0 != ((1 << i) & mask));
}

    bool
any_boolPack (boolPack a)
{
    return (0 != _mm_movemask_ps (a));
}

    bool
all_boolPack (boolPack a)
{
    return (0xF == _mm_movemask_ps (a));
}

    uint
count_boolPack (boolPack a)
{
    static const uint bits[] =
    {
        0, 1, 1, 2,
        1, 2, 2, 3,
        1, 2, 2, 3, 
        2, 3, 3, 4
    };
    return bits[_mm_movemask_ps (a)];
}

    boolPack
fill_boolPack (bool t)
{
    if (t)  return _mm_castsi128_ps (fill_uintPack ((uint)-1));
    else    return false_boolPack ();
}

    boolPack
blend_boolPack (boolPack a, boolPack b, boolPack f)
{
    return or_boolPack (and_boolPack (f, a),
                        andnot_boolPack (f, b));
}

    uintPack
pack_uintPack (const uint* src)
{
    union {
        uintPack u;
        uint v[realPackSz];
    } x;
    Op_0( uint, realPackSz, x.v , src );
    return x.u;
}

    void
scat_uintPack (uint* dst, uintPack src)
{
    union {
        uintPack u;
        uint v[realPackSz];
    } x;
    x.u = src;
    Op_0( uint, realPackSz, dst , x.v );
}

    void
cfill_uintPack (uintPack* dst, uint src, boolPack mask)
{
    uintPack x;
    x = fill_uintPack (src);
    _mm_maskmoveu_si128 (x,
                         _mm_castps_si128 (mask),
                         (char*)dst);
}

    boolPack
eql1_uintPack (uintPack a, uint b)
{
    return _mm_castsi128_ps (_mm_cmpeq_epi32 (a, fill_uintPack (b)));
}

    boolPack
less1_uintPack (uintPack a, uint b)
{
    return _mm_castsi128_ps (_mm_cmplt_epi32 (a, fill_uintPack (b)));
}

    void
cmove_realPack (realPack* dst, realPack src, boolPack mask)
{
    _mm_maskmoveu_si128 (_mm_castps_si128 (src),
                         _mm_castps_si128 (mask),
                         (char*)dst);
}

    realPack
abs_realPack (realPack x)
{
    const realPack signmask =
        _mm_castsi128_ps (_mm_set1_epi32 (0x80000000));
    return _mm_andnot_ps (signmask, x);
}

    void
pack_PointPack (PointPack* dst, const Point* src)
{
    uint dim, i;
    UUFor( dim, NDimensions, i, realPackSz )
        dst->vcoords[dim][i] = src[i].coords[dim];
}

    void
fill_PointPack (PointPack* dst, const Point* src)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = fill_realPack (src->coords[dim]);
}

    void
scat_PointPack (Point* dst, const PointPack* src)
{
    uint dim, i;
    UUFor( dim, NDimensions, i, realPackSz )
        dst[i].coords[dim] = src->vcoords[dim][i];
}

    void
copy_PointPack (PointPack* dst, const PointPack* src)
{
    uint dim;
    UFor( dim, NDimensions )  dst->coords[dim] = src->coords[dim];
}

    void
summ_PointPack (PointPack* dst, const PointPack* a, const PointPack* b)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = summ_realPack (a->coords[dim], b->coords[dim]);
}

    void
diff_PointPack (PointPack* dst, const PointPack* a, const PointPack* b)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = diff_realPack (a->coords[dim], b->coords[dim]);
}

    void
prod_PointPack (PointPack* dst, const PointPack* src, realPack x)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = prod_realPack (src->coords[dim], x);
}

    void
quot_PointPack (PointPack* dst, const PointPack* src, realPack x)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = quot_realPack (src->coords[dim], x);
}

    void
reci_PointPack (PointPack* dst, const PointPack* src)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = reci_realPack (src->coords[dim]);
}

    void
rayto_PointPack (PointPack* dst,
                 const PointPack* origin,
                 const PointPack* direct,
                 realPack mag)
{
    uint dim;
    UFor( dim, NDimensions )
    {
        dst->coords[dim] =
            summ_realPack (origin->coords[dim],
                           prod_realPack (direct->coords[dim], mag));
    }
}

    realPack
dot_PointPack (const PointPack* a, const PointPack* b)
{
    uint i;
    realPack sum;
    sum = zero_realPack ();
    UFor( i, NDimensions )
    {
        realPack x;
        x = prod_realPack (a->coords[i], b->coords[i]);
        sum = summ_realPack (sum, x);
    }
    return sum;
}

    realPack
dot1_PointPack (const PointPack* a, const Point* b)
{
    uint i;
    realPack sum;
    sum = zero_realPack ();
    UFor( i, NDimensions )
    {
        realPack x;
        x = prod_realPack (a->coords[i], fill_realPack (b->coords[i]));
        sum = summ_realPack (sum, x);
    }
    return sum;
}

    realPack
magnitude_PointPack (const PointPack* a)
{
    return _mm_sqrt_ps (dot_PointPack (a, a));
}

    void
normalize_PointPack (PointPack* dst, const PointPack* src)
{
    quot_PointPack (dst, src, magnitude_PointPack (src));
}

    boolPack
inside_pack_BBox (const BBox* box, const PointPack* point)
{
    boolPack inside;
    uint i;

    inside = fill_boolPack (false);
    UFor( i, NDimensions )
    {
        realPack lo, hi;
        boolPack intmp;
        lo = fill_realPack (box->min.coords[i]);
        hi = fill_realPack (box->max.coords[i]);
        intmp = and_boolPack (leql_realPack (lo, point->coords[i]),
                              geql_realPack (hi, point->coords[i]));
                                           
        inside = and_boolPack (inside, intmp);
    }
    return inside;
}

    boolPack
hit_pack_BarySimplex (realPack* restrict ret_dist,
                      const PointPack* restrict origin,
                      const PointPack* restrict direct,
                      const BarySimplex* restrict elem)
{
    PointPack isect;
    realPack dist, dot, bcoord_sum;
    boolPack valid;
    uint i;

    dist = summ_realPack (dot1_PointPack (origin, &elem->plane.normal),
                          fill_realPack (elem->plane.offset));
    dot = dot1_PointPack (direct, &elem->plane.normal);
    dist = quot_realPack (dist, dot);

    valid = leql_realPack (dist, zero_realPack ());

    dist = abs_realPack (dist);
    rayto_PointPack (&isect, origin, direct, dist);
    *ret_dist = dist;
        /* /dist/ means something different below!*/
    
    bcoord_sum = zero_realPack ();
    UFor( i, NDimensions-1 )
    {
        const Plane* bplane;
        bplane = &elem->barys[i];

        dist = summ_realPack (dot1_PointPack (&isect, &bplane->normal),
                              fill_realPack (bplane->offset));

        valid = and_boolPack (valid,
                              geql_realPack (dist, zero_realPack ()));
        bcoord_sum = summ_realPack (bcoord_sum, dist);
    }

    return and_boolPack (valid,
                         leql_realPack (bcoord_sum, fill_realPack (1)));
}

    void
test_pack_intersections (uintPack* ret_hit,
                         realPack* ret_mag,
                         const PointPack* restrict origin,
                         const PointPack* restrict direct,
                         uint nelemidcs,
                         const uint* restrict elemidcs,
                         const BarySimplex* restrict simplices)
{
    uint i;
    bool hit_any = false;
    uintPack hit_idx;
    realPack hit_mag;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;

    UFor( i, nelemidcs )
    {
        boolPack didhit;
        uint tmp_hit;
        realPack tmp_mag;

        tmp_hit = elemidcs[i];

        didhit = hit_pack_BarySimplex (&tmp_mag, origin, direct,
                                       &simplices[tmp_hit]);

        if (any_boolPack (didhit))
        {
            hit_any = true;
            didhit = and_boolPack (didhit,
                                   leql_realPack (tmp_mag, hit_mag));

            cmove_realPack (&hit_mag, tmp_mag, didhit);
            cfill_uintPack (&hit_idx, tmp_hit, didhit);
        }
    }

    if (hit_any)
    {
        *ret_hit = hit_idx;
        *ret_mag = hit_mag;
    }
}

