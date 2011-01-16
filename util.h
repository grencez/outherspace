
#ifndef UTIL_H_
#define UTIL_H_

#include <stdlib.h>

#define UFor( i, bel )  for (i = 0; i < bel; ++i)

    /* typedef double real; */
typedef float real;
typedef unsigned uint;
typedef char tristate;
typedef unsigned char byte;

#ifndef COMPILER_HAS_BOOL
typedef byte bool;
#define true 1
#define false 0
#endif

uint index_of (const void* e, const void* arr, size_t size);
tristate compare_real (real a, real b);
tristate signum_real (real a);

#endif

