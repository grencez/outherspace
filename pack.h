
#ifndef PACK_H_
#define PACK_H_

#include "simplex.h"
#include "space.h"
#include "util.h"

#include <xmmintrin.h>
typedef __m128 boolPack;
typedef __m128i uintPack;
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
#define false_boolPack _mm_setzero_ps

#define fill_uintPack _mm_set1_epi32
#define eql_uintPack _mm_set1_epi32

#define less_realPack _mm_cmplt_ps
#define leql_realPack _mm_cmple_ps
#define geql_realPack _mm_cmpge_ps

#define fill_realPack _mm_set_ps1
#define summ_realPack _mm_add_ps
#define diff_realPack _mm_sub_ps
#define prod_realPack _mm_mul_ps
#define quot_realPack _mm_div_ps
#define reci_realPack _mm_rcp_ps
#define zero_realPack _mm_setzero_ps
#define scat_realPack _mm_store_ps

boolPack
pack_boolPack (const bool* src);
void
scat_boolPack (bool* dst, boolPack src);
bool
any_boolPack (boolPack a);
bool
all_boolPack (boolPack a);
uint
count_boolPack (boolPack a);
boolPack
fill_boolPack (bool t);
boolPack
blend_boolPack (boolPack a, boolPack b, boolPack f);
uintPack
pack_uintPack (const uint* src);
void
scat_uintPack (uint* dst, uintPack src);
void
cfill_uintPack (uintPack* dst, uint src, boolPack mask);
boolPack
eql1_uintPack (uintPack a, uint b);
boolPack
less1_uintPack (uintPack a, uint b);
void
cmove_realPack (realPack* dst, realPack src, boolPack mask);
realPack
abs_realPack (realPack x);
void
pack_PointPack (PointPack* dst, const Point* src);
void
fill_PointPack (PointPack* dst, const Point* src);
void
scat_PointPack (Point* dst, const PointPack* src);
void
copy_PointPack (PointPack* dst, const PointPack* src);
void
summ_PointPack (PointPack* dst, const PointPack* a, const PointPack* b);
void
diff_PointPack (PointPack* dst, const PointPack* a, const PointPack* b);
void
prod_PointPack (PointPack* dst, const PointPack* src, realPack x);
void
quot_PointPack (PointPack* dst, const PointPack* src, realPack x);
void
reci_PointPack (PointPack* dst, const PointPack* src);
void
rayto_PointPack (PointPack* dst,
                 const PointPack* origin,
                 const PointPack* direct,
                 realPack mag);
realPack
dot_PointPack (const PointPack* a, const PointPack* b);
realPack
dot1_PointPack (const PointPack* a, const Point* b);
realPack
magnitude_PointPack (const PointPack* a);
void
normalize_PointPack (PointPack* dst, const PointPack* src);
boolPack
inside_pack_BBox (const BBox* box, const PointPack* point);
boolPack
hit_pack_BarySimplex (realPack* restrict ret_dist,
                      const PointPack* restrict origin,
                      const PointPack* restrict direct,
                      const BarySimplex* restrict elem);
void
test_pack_intersections (uintPack* ret_hit,
                         realPack* ret_mag,
                         const PointPack* restrict origin,
                         const PointPack* restrict direct,
                         uint nelemidcs,
                         const uint* restrict elemidcs,
                         const BarySimplex* restrict simplices);

#ifdef IncludeC
#include "pack.c"
#endif
#endif

