
#ifndef UTIL_H_
#ifndef __OPENCL_VERSION__
#define UTIL_H_

#include <float.h>
#include <limits.h>
#include <stdlib.h>

#define AllocT( Type, capacity ) \
    (((capacity) == 0) ? (Type*) 0 : \
     (Type*) malloc ((capacity) * sizeof (Type)))

typedef unsigned int uint;
typedef unsigned char byte;

#ifndef COMPILER_HAS_BOOL
typedef byte bool;
#define true 1
#define false 0
#endif

#if __STDC_VERSION__ < 199901L
#define restrict __restrict
#endif

#ifdef BENCHMARKING
#ifndef NDEBUG
#define NDEBUG
#endif
#endif
#define INCLUDE_SOURCE

#define __global

#else  /* #ifndef __OPENCL_VERSION__ */
#define assert (void)
#define static
#endif  /* #ifdef __OPENCL_VERSION__ */


#define UFor( i, bel )  for (i = 0; i < bel; ++i)

#if 0
typedef double real;
#define Max_real DBL_MAX
#define Min_real DBL_MIN

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#else
typedef float real;
#define Max_real FLT_MAX
#define Min_real FLT_MIN

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#endif

#define Max_uint UINT_MAX

typedef int tristate;

bool even_uint (uint a);
tristate compare_real (real a, real b);
tristate signum_real (real a);
tristate mul_signum (tristate a, tristate b);


#ifndef __OPENCL_VERSION__
uint index_of (const void* e, const void* arr, size_t size);
void array_set (void* arr, uint i, const void* e, size_t size);
char* strto_uint (uint* ret, const char* in);
char* strto_real (real* ret, const char* in);
#ifdef INCLUDE_SOURCE
#include "util.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

