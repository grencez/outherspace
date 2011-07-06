
#ifndef __OPENCL_VERSION__
#include "util.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#ifdef DistribCompute
#include <mpi.h>
#endif

#ifdef _OPENMP
#include <omp.h>
#else
#include <time.h>
#endif

uint index_of (const void* e, const void* arr, size_t size)
{
    return ((size_t) e - (size_t) arr) / size;
}

void array_set (void* arr, uint i, const void* e, size_t size)
{
    memcpy ((void*) ((size_t) arr + ((size_t) i * size)), e, size);
}

    void
array_cpy (void* dst, const void* src, uint lo, uint count, size_t size)
{
    if (size > 0)
        memcpy ((void*) ((size_t) dst + ((size_t) lo * size)),
                src,
                count * size);
}

char* strto_uint (uint* ret, const char* in)
{
    unsigned long v;
    char* out = 0;

    assert (ret);
    assert (in);
    v = strtoul (in, &out, 10);

    if (v > Max_uint)  out = 0;
    if (out == in)  out = 0;
    if (out)  *ret = (uint) v;
    return out;
}

char* strto_real (real* ret, const char* in)
{
    double v;
    char* out = 0;

    assert (ret);
    assert (in);
    v = strtod (in, &out);

    if (out == in)  out = 0;
    if (out)  *ret = (real) v;
    return out;
}

const char* strskip_ws (const char* line)
{
    uint i;
    i = strspn (line, " \t\v\r\n");
    return &line[i];
}

void strstrip_eol (char* line)
{
    uint i;
    i = strcspn (line, "\r\n");
    line[i] = '\0';
}

uint readin_whitesep (char* buf, FILE* in, uint capacity, uint len)
{
    assert (capacity > len);
    if (capacity-1 != len)
        memmove (buf, &buf[len], (capacity - len - 1) * sizeof(char));
    len = capacity - len - 1;
    len += fread (&buf[len], sizeof(char), capacity - len - 1, in);
    assert (len < capacity);
    buf[len] = '\0';

    if (len == 0)  return 0;

        /* Could have partial data, step back.*/
    if (!isspace (buf[len]))
        while (len > 0 && !isspace (buf[len-1]))
            len -= 1;

        /* Skip trailing space.*/
    while (len > 0 && isspace (buf[len-1]))
        len -= 1;

    return len;
}

    FILE*
fopen_path (const char* pathname, const char* filename, const char* mode)
{
    uint len;
    char* path;
    FILE* f;
    len = strlen (pathname) + 1 + strlen (filename);
    path = AllocT( char, len+1 );
    sprintf (path, "%s/%s", pathname, filename);
    f = fopen (path, mode);
    free (path);
    return f;
}

real monotime ()
{
#if defined(DistribCompute)
    static double t0 = -1;
    if (t0 < 0)
    {
        t0 = MPI_Wtime ();
        return 0;
    }
    else
    {
        return (real) (MPI_Wtime () - t0);
    }
#elif defined(_OPENMP)
    return (real) omp_get_wtime ();
#else
    return ((real) clock ()) / CLOCKS_PER_SEC;
#endif
}

void assert_status (int stat, const char* msg, const char* file, int line)
{
    if (stat != 0)
    {
#ifdef DistribCompute
        int proc;
        MPI_Comm_rank (MPI_COMM_WORLD, &proc);
        fprintf (stderr, "Process %d assertion failure! ", proc);
#endif
        fprintf (stderr, "%s:%d - Bad status: %d - %s\n",
                 file, line, stat, msg);
    }
    assert (stat == 0);
}
#endif  /* #ifndef __OPENCL_VERSION__ */


bool even_uint (uint a)
{
    return 0 == (a & 1);
}

    uint
ceil_uint (uint a, uint b)
{
    return Ceil_uint (a, b);
}

    uint
log2_uint (uint a)
{
    uint p = 0;
    assert (a > 0);
    do
    {
        a >>= 1;
        ++p;
    }
    while (a > 0);
    return p-1;
}

    uint
exp2_uint (uint p)
{
    assert (p < 32);
    return 1 << p;
}

    void
swap_uint (uint* x, uint* y)
{
    uint tmp = *x;
    *x = *y;
    *y = tmp;
}

    void
swap_real (real* x, real* y)
{
    real tmp = *x;
    *x = *y;
    *y = tmp;
}

tristate compare_real (real a, real b)
{
    if (a > b)  return  1;
    if (a < b)  return -1;
    return 0;
}

real match_real (real a, real b)
{
    if (a < 0)
    {
        if (b > 0)  b = - b;
        if (a < b)  b = a;
    }
    else
    {
        if (b < 0)  b = - b;
        if (a > b)  b = a;
    }
    return b;
}

    real
clamp_real (real x, real lo, real hi)
{
    assert (lo <= hi);
    if (x < lo)  return lo;
    if (x > hi)  return hi;
    return x;
}

    real
atan2_real (real y, real x)
{
    real a = 0;
        /* assert (y==y); */
        /* assert (x==x); */
    if (x > -Epsilon_real && x < Epsilon_real)
    {
            /* Equality case undefined... meh.*/
        if (y >= 0)  return   M_PI / 2;
        else         return - M_PI / 2;
    }

    if (x < 0)
    {
        if (y >= 0)  a =   M_PI;
        else         a = - M_PI;
    }
    return a + atan (y / x);
}

real absolute_error (real expect, real result)
{
    return result - expect;
}

real relative_error (real expect, real result, real large)
{
    real err;
    err = absolute_error (expect, result);
    if (- Epsilon_real < err && err < Epsilon_real)
        return err;

    return err / match_real (expect, large);
}

    real
approx_eql (real expect, real result, real large, real mul)
{
    return mul*Epsilon_real >= fabs (relative_error (expect, result, large));
}

tristate signum_real (real a)
{
    return compare_real (a, 0);
}

tristate mul_signum (tristate a, tristate b)
{
    if (a == 0 || b == 0)
        return 0;
    if (a == b)
        return 1;
    return -1;
}

