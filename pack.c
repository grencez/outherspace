
#include "pack.h"

    boolPack
blend_boolPack (boolPack a, boolPack b, boolPack f)
{
    return or_boolPack (and_boolPack (f, a),
                        andnot_boolPack (f, b));
}
    realPack
abs_realPack (realPack x)
{
    const realPack signmask =
        _mm_castsi128_ps (_mm_set1_epi32 (0x80000000));
    return _mm_andnot_ps (signmask, x);
}

    void
fill_PointPack (PointPack* dst, const Point* src)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = fill_realPack (src->coords[dim]);
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
        dst->coords[dim] = _mm_add_ps (a->coords[dim], b->coords[dim]);
}

    void
diff_PointPack (PointPack* dst, const PointPack* a, const PointPack* b)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = _mm_sub_ps (a->coords[dim], b->coords[dim]);
}

    void
mult_PointPack (PointPack* dst, const PointPack* src, realPack x)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = _mm_mul_ps (src->coords[dim], x);
}

    void
divi_PointPack (PointPack* dst, const PointPack* src, realPack x)
{
    uint dim;
    UFor( dim, NDimensions )
        dst->coords[dim] = _mm_div_ps (src->coords[dim], x);
}

    realPack
dot_PointPack (const PointPack* a, const PointPack* b)
{
    uint i;
    realPack sum;
    sum = mult_realPack (a->coords[0], b->coords[0]);
    UFor( i, NDimensions-1 )
    {
        realPack x;
        x = mult_realPack (a->coords[i+1], b->coords[i+1]);
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
    divi_PointPack (dst, src, magnitude_PointPack (src));
}


    boolPack
hit_pack_BarySimplex (realPack* restrict ret_dist,
                      const PointPack* restrict origin,
                      const PointPack* restrict direct,
                      const BarySimplex* restrict elem)
{
    PointPack isect;
    PointPack normal;
    realPack dist, dot, bcoord_sum;
    boolPack valid;
    uint i;

    fill_PointPack (&normal, &elem->plane.normal);

    dist = summ_realPack (dot_PointPack (&normal, origin),
                          fill_realPack (elem->plane.offset));
    dot = dot_PointPack (&normal, direct);
        dist = divi_realPack (dist, dot);

    valid = less_realPack (dist, zero_realPack ());

    mult_PointPack (&isect, direct, dist);
    summ_PointPack (&isect, &isect, origin);
    *ret_dist = dist;
    
    bcoord_sum = zero_realPack ();
    UFor( i, NDimensions-1 )
    {
        fill_PointPack (&normal, &elem->barys[i].normal);

        dist = summ_realPack (dot_PointPack (&normal, &isect),
                              fill_realPack (elem->barys[i].offset));

        valid = and_boolPack (valid,
                              geql_realPack (dist, zero_realPack ()));
        bcoord_sum = summ_realPack (bcoord_sum, dist);
    }

    return and_boolPack (valid,
                         leql_realPack (bcoord_sum, zero_realPack ()));
}

