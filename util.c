
#ifndef __OPENCL_VERSION__
#include "util.h"

#include <assert.h>
#include <stdio.h>
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

