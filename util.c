
#ifndef __OPENCL_VERSION__
#include "util.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <string.h>

#ifdef DistribCompute
#include <mpi.h>
#endif

#if defined(__INTEL_COMPILER)
#include <time.h>
#elif defined(_OPENMP)
#include <omp.h>
#else
#include <time.h>
#endif

static const char WhiteSpaceChars[] = " \t\v\r\n";

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

    void*
array_dup (const void* src, uint count, size_t size)
{
    void* dst;
    if (count * size == 0)  return 0;
    dst = malloc (count * size);
    array_cpy (dst, src, 0, count, size);
    return dst;
}

    void*
array_cat (void* dst, const void* src, uint* end, uint count, size_t size)
{
    uint n;
    n = *end + count;
    dst = realloc (dst, n * size);
    array_cpy (dst, src, *end, count, size);
    *end = n;
    return dst;
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

    uint
strcount_ws (const char* s)
{
    return strspn (s, WhiteSpaceChars);
}

    uint
strcount_non_ws (const char* s)
{
    return strcspn (s, WhiteSpaceChars);
}

const char* strskip_ws (const char* line)
{
    uint i;
    i = strspn (line, WhiteSpaceChars);
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

    /* Concatenate strings.*/
    char*
cat_strings (uint n, const char* const* a)
{
    uint i, len;
    char* s;

    len = 0;
    UFor( i, n )  len += strlen (a[i]);

    s = AllocT( char, len+1 );

    len = 0;
    UFor( i, n )
    {
        uint count;
        count = strlen (a[i]);
        array_cpy (s, a[i], len, count, sizeof(char));
        len += count;
    }
    s[len] = '\0';
    return s;
}

    /* Create a full filepath from a pathname and filename.*/
    char*
cat_filepath (const char* pathname, const char* filename)
{
    const char* parts[3];
    parts[0] = pathname;
    parts[1] = "/";
    parts[2] = filename;
    return cat_strings (3, parts);
}

    FILE*
fopen_path (const char* pathname, const char* filename, const char* mode)
{
    char* path;
    FILE* f;
    path = cat_filepath (pathname, filename);
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
#elif defined(__INTEL_COMPILER)
        /* TODO: ICC's omp_get_wtime() doesn't work for some reason.
         * This gives the wrong result with OpenMP on.
         */
    return ((real) clock ()) / CLOCKS_PER_SEC;
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

    uint
incmod_uint (uint a, uint b, uint m)
{
    a += b;
    if (a >= m)  a -= m;
    assert (a < m);
    return a;
}

    uint
decmod_uint (uint a, uint b, uint m)
{
    if (a < b)  a += m - b;
    else        a -= b;
    assert (a < m);
    return a;
}

    uint
assoc_index (uint n, uint a, uint b)
{
    uint idx;
    assert (a < b);
    idx = a * (n - 1) + (b - 1) - a * (a + 1) / 2;
    assert (idx < n * (n-1) / 2);
    return idx;
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
abs_real (real x)
{
    return (x >= 0) ? x : -x;
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

    /** Really stupid random function... does the trick though.**/
    real
random_real (srand_t* seed)
{
        /* RAND_MAX assumed to be 32767 */
    const uint max = 32767;
    real v;
    *seed = *seed * 1103515245 + 12345;
    v = (real) ((*seed/65536) % (max+1)) / (max+1);
    assert (v >= 0);
    assert (v < 1);
    return v;
}

    uint
random_uint (srand_t* seed, uint n)
{
    return (uint) (n * random_real (seed));
}

    bool
random_bool (srand_t* seed)
{
    return random_real (seed) < .5;
}

