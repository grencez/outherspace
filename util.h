
#ifndef UTIL_H_
#define UTIL_H_

#include <float.h>
#include <limits.h>
#include <stdlib.h>

#define UFor( i, bel )  for (i = 0; i < bel; ++i)

#if 0
typedef double real;
#define Max_real DBL_MAX
#define Min_real DBL_MIN
#else
typedef float real;
#define Max_real FLT_MAX
#define Min_real FLT_MIN
#endif

typedef unsigned uint;
#define Max_uint UINT_MAX

typedef char tristate;
typedef unsigned char byte;

#ifndef COMPILER_HAS_BOOL
typedef byte bool;
#define true 1
#define false 0
#endif

#define AllocT( Type, capacity ) \
    (((capacity) == 0) ? (Type*) 0 : \
     (Type*) malloc ((capacity) * sizeof (Type)))

uint index_of (const void* e, const void* arr, size_t size);
void array_set (void* arr, uint i, const void* e, size_t size);
bool even_uint (uint a);
tristate compare_real (real a, real b);
tristate signum_real (real a);
tristate mul_signum (real a, real b);
char* strto_uint (uint* ret, const char* in);
char* strto_real (real* ret, const char* in);

#include "util.c"
#endif

