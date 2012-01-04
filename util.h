
#ifndef UTIL_H_
#ifndef __OPENCL_VERSION__
#define UTIL_H_

#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
    /* Disable warning: 'fopen' unsafe, use fopen_s instead */
    /* REF CMakeLists.txt: Define _CRT_SECURE_NO_WARNINGS */

    /* Disable: conditional expression is constant */
# pragma warning (disable : 4127)
    /* Disable: conversion from 'uint' to 'real' */
# pragma warning (disable : 4244)
    /* Disable: conversion from 'double' to 'float' */
# pragma warning (disable : 4305)
#endif

#define Stringify(a) #a
#define Concatify(a,b) a ## b
#define Concatify2(a,b)  Concatify(a,b)
#define StringifyPath(a,b) Stringify(a/b)

#ifndef EmbedPathnamePfx
#define EmbedPathnamePfx .
#endif
#ifdef EmbedFilenamePfx
#define EmbedInclude(s) \
    StringifyPath(EmbedPathnamePfx,Concatify2(EmbedFilenamePfx,s.embed.h))
#else  /* ^^^ defined(EmbedFilenamePfx) */
#define EmbedInclude(s) \
    StringifyPath(EmbedPathnamePfx,s.embed.h)
#endif  /* !defined(EmbedFilenamePfx) */


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
        
#define AccepTok( line, tok ) \
    ((0 == strncmp ((line), (tok), strlen(tok))) \
     ? ((line) = &(line)[strlen(tok)]) \
     : 0)

#define Ceil_uint( a, b ) \
    (((a) + (b) - 1) / (b))

#define ArraySz( a )  sizeof(a) / sizeof(*a)

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
#else  /* ^^^ !defined(__OPENCL_VERSION__) */
#define assert (void)
#define static
#endif  /* defined(__OPENCL_VERSION__) */


#define UFor( i, bel )  for (i = 0; i < (bel); ++i)
#define UFr(i, bel, body)  do \
{ \
    uint i; \
    UFor( i, bel ) \
    { body; } \
} while (0)

    /* Does not scope /j/.*/
#define UUFor( i, ibel, j, jbel )  UFor( i, ibel )  UFor( j, jbel )


#if 0
typedef double real;
#define Max_real DBL_MAX
#define Min_real (-DBL_MAX)
#define Small_real DBL_MIN
#define Epsilon_real DBL_EPSILON
#define realPackSz 2
#define GL_REAL GL_DOUBLE

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
#define GL_REAL GL_FLOAT

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
uint
max_uint (uint a, uint b);
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

#define AssertEq( expect, result )  assert ((expect) == (result))
#define AssertEqA( N, expect, result, off_expect, off_result )  do \
{ \
    uint AssertEqA_i; \
    uint AssertEqA_off_expect; \
    uint AssertEqA_off_result; \
    AssertEqA_off_expect = off_expect; \
    AssertEqA_off_result = off_result; \
    UFor( AssertEqA_i, N ) \
    { \
        AssertEq( (expect)[AssertEqA_i + AssertEqA_off_expect], \
                  (result)[AssertEqA_i + AssertEqA_off_result] ); \
    } \
} while (0)

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
bool
readin_files (uint nfiles, uint* files_nbytes, byte** files_bytes,
              const char* pathname,
              const char* const* files);
real monotime ();
void assert_status (int stat, const char* msg, const char* file, int line);
#ifdef INCLUDE_SOURCE
#include "util.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

