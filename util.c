
#include "util.h"

uint index_of (const void* e, const void* arr, size_t size)
{
    return ((size_t) e - (size_t) arr) / size;
}

tristate compare_real (real a, real b)
{
    if (a > b)  return  1;
    if (a < b)  return -1;
    return 0;
}

tristate signum_real (real a)
{
    return compare_real (a, 0);
}

tristate mul_signum (real a, real b)
{
    if (a == 0 || b == 0)
        return 0;
    if (a == b)
        return 1;
    return -1;
}

