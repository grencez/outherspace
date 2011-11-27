
#ifndef UTIL_H_
#ifndef __OPENCL_VERSION__
#define UTIL_H_

#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>


#define AllocT( Type, capacity ) \
    (((capacity) == 0) ? (Type*) 0 : \
     (Type*) malloc ((capacity) * sizeof (Type)))

#if 0
#define ResizeT( Type, arr, old_capacity, new_capacity )  do \
{ \
    if (old_capacity == 0)  (arr) = AllocT( Type, new_capacity ); \
    else if (new_capacity == 0)  free (arr); \
    else  (arr) = (Type*) realloc (arr, (new_capacity) * sizeof (Type)); \
} while (0)
#else
#define ResizeT( Type, arr, capacity ) \
    ((arr) = (Type*) realloc (arr, (capacity) * sizeof (Type)))
#endif

#define CopyT( Type, dst, src, lo, count ) \
    (array_cpy (dst, src, lo, count, sizeof (Type)))

#define DuplicaT( Type, src, count ) \
    ((Type*) array_dup (src, count, sizeof (Type)))

#define ConcaT( Type, dst, src, end, count ) \
    ((dst) = (Type*) array_cat (dst, src, &end, count, sizeof (Type)))
        

#define Ceil_uint( a, b ) \
    (((a) + (b) - 1) / (b))

typedef unsigned int uint;
typedef unsigned char byte;

typedef unsigned long srand_t;

#ifndef uint32
#define uint32 uint
#endif

#define NBitsInByte 8

#ifndef COMPILER_HAS_BOOL
typedef byte bool;
#define true 1
#define false 0
#endif

#if __STDC_VERSION__ < 199901L
#define restrict __restrict
#endif

#define __global

#else  /* #ifndef __OPENCL_VERSION__ */
#define assert (void)
#define static
#endif  /* #ifdef __OPENCL_VERSION__ */


#define UFor( i, bel )  for (i = 0; i < (bel); ++i)

    /* Does not scope /j/.*/
#define UUFor( i, ibel, j, jbel )  UFor( i, ibel )  UFor( j, jbel )


#if 0
typedef double real;
#define Max_real DBL_MAX
#define Min_real (-DBL_MAX)
#define Small_real DBL_MIN
#define Epsilon_real DBL_EPSILON
#define realPackSz 2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef DistribCompute
#define MPI_real MPI_DOUBLE
#endif

#else
typedef float real;
#define Max_real FLT_MAX
#define Min_real (-FLT_MAX)
#define Small_real FLT_MIN
#define Epsilon_real FLT_EPSILON
#define realPackSz 4

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifdef DistribCompute
#define MPI_real MPI_FLOAT
#endif

#endif

#define NColors 3

#define Max_uint UINT_MAX

typedef int tristate;

    /* Note: All functions below are ideally inline'd!*/
bool even_uint (uint a);
uint
ceil_uint (uint a, uint b);
uint
log2_uint (uint a);
uint
exp2_uint (uint p);
uint
incmod_uint (uint a, uint b, uint m);
uint
decmod_uint (uint a, uint b, uint m);
uint
assoc_index (uint n, uint a, uint b);
void
swap_uint (uint* x, uint* y);
void
swap_real (real* x, real* y);
tristate compare_real (real a, real b);
real match_real (real a, real b);
real
clamp_real (real x, real lo, real hi);
real
abs_real (real x);
real
atan2_real (real y, real x);
real absolute_error (real expect, real result);
real relative_error (real expect, real result, real large);
real
approx_eql (real expect, real result, real large, real mul);
tristate signum_real (real a);
tristate mul_signum (tristate a, tristate b);
real
random_real (srand_t* seed);
uint
random_uint (srand_t* seed, uint n);
bool
random_bool (srand_t* seed);

#ifdef NDEBUG
#define AssertApprox( expect, result, large, mul )
#define AssertStatus( stat, msg )
#else
#define AssertApprox( expect, result, large, mul ) \
    assert (approx_eql (expect, result, large, mul))
#define AssertStatus( stat, msg ) \
    assert_status (stat, msg, __FILE__, __LINE__)
#endif

#ifndef __OPENCL_VERSION__
uint index_of (const void* e, const void* arr, size_t size);
void array_set (void* arr, uint i, const void* e, size_t size);
void
array_cpy (void* dst, const void* src, uint lo, uint count, size_t size);
void*
array_dup (const void* src, uint count, size_t size);
void*
array_cat (void* dst, const void* src, uint* end, uint count, size_t size);
char* strto_uint (uint* ret, const char* in);
char* strto_real (real* ret, const char* in);
uint strcount_ws (const char* s);
uint strcount_non_ws (const char* s);
const char* strskip_ws (const char* line);
void strstrip_eol (char* line);
uint readin_whitesep (char* buf, FILE* in, uint capacity, uint len);
char*
cat_strings (uint n, const char* const* a);
char*
cat_filepath (const char* pathname, const char* filename);
bool
strends_with (const char* str, const char* sfx);
FILE*
fopen_path (const char* pathname, const char* filename, const char* mode);
real monotime ();
void assert_status (int stat, const char* msg, const char* file, int line);
#ifdef INCLUDE_SOURCE
#include "util.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

