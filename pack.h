
#ifndef PACK_H_
#define PACK_H_

#include "simplex.h"
#include "space.h"
#include "util.h"

#include <xmmintrin.h>
typedef __m128 boolPack;
typedef __m128 realPack;


union point_pack_union
{
    real vcoords[NDimensions][realPackSz];
    realPack coords[NDimensions];
};
typedef union point_pack_union PointPack;

#define xor_boolPack _mm_xor_ps
#define and_boolPack _mm_and_ps
#define or_boolPack _mm_or_ps
#define andnot_boolPack _mm_andnot_ps

#define less_realPack _mm_cmplt_ps
#define leql_realPack _mm_cmple_ps
#define geql_realPack _mm_cmpge_ps

#define fill_realPack _mm_set_ps1
#define summ_realPack _mm_add_ps
#define diff_realPack _mm_sub_ps
#define mult_realPack _mm_mul_ps
#define divi_realPack _mm_div_ps
#define zero_realPack _mm_setzero_ps

boolPack
blend_boolPack (boolPack a, boolPack b, boolPack f);
realPack
abs_realPack (realPack x);
void
fill_PointPack (PointPack* dst, const Point* src);
void
copy_PointPack (PointPack* dst, const PointPack* src);
void
summ_PointPack (PointPack* dst, const PointPack* a, const PointPack* b);
void
diff_PointPack (PointPack* dst, const PointPack* a, const PointPack* b);
void
mult_PointPack (PointPack* dst, const PointPack* src, realPack x);
void
divi_PointPack (PointPack* dst, const PointPack* src, realPack x);
realPack
dot_PointPack (const PointPack* a, const PointPack* b);
realPack
magnitude_PointPack (const PointPack* a);
void
normalize_PointPack (PointPack* dst, const PointPack* src);
boolPack
hit_pack_BarySimplex (realPack* restrict ret_dist,
                      const PointPack* restrict origin,
                      const PointPack* restrict direct,
                      const BarySimplex* restrict elem);

#ifdef INCLUDE_SOURCE
#include "pack.c"
#endif
#endif

