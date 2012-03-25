
#include "order.h"
#include "bitstring.h"

#include <assert.h>

static uint
mid5_reals (const real a[5])
{
    uint i;
    real b[5];

    UFor( i, 5 )  b[i] = a[i];
    sort_few_reals (5, b);
    UFor( i, 5 )
        if (a[i] == b[2])
            return i;

    Claim( 0 );
    return 0;
}

    void
fill_minimal_unique (uint* a, uint n)
{
    uint i;
    UFor( i, n )  a[i] = i;
}

    bool
minimal_unique (uint n, const uint* a)
{
    uint i;
    BitString* hits;
    bool pred = true;

    hits = alloc_BitString (n, false);

    UFor( i, n )
    {
        Claim( a[i] < n );
        if (set1_BitString (hits, a[i]))
            pred = false;
    }

    free_BitString (hits);
    return pred;
}

    /* Insertion sort.*/
    void
sort_few_uints (uint n, uint* a)
{
    uint j;
    for (j = 1; j < n; ++j)
    {
        uint i, key;
        key = a[j];
        i = j;
        while (i > 0 && a[i-1] > key)
        {
            a[i] = a[i-1];
            i -= 1;
        }
        a[i] = key;
    }
}

    /* Insertion sort.*/
    void
sort_few_reals (uint n, real* a)
{
    uint j;
    for (j = 1; j < n; ++j)
    {
        uint i;
        real key;
        key = a[j];
        i = j;
        while (i > 0 && a[i-1] > key)
        {
            a[i] = a[i-1];
            i -= 1;
        }
        a[i] = key;
    }
}

    /* Insertion sort.*/
static inline
    void
sort_few_indexed_reals (uint n, uint* indices, const real* membs)
{
    uint j;
    for (j = 1; j < n; ++j)
    {
        uint i = j;
        const uint keyi = indices[j];
        const real key = membs[keyi];
        while (i > 0 && membs[indices[i-1]] > key)
        {
            indices[i] = indices[i-1];
            i -= 1;
        }
        indices[i] = keyi;
    }
}

    /*
     * Here are the inputs and meanings by example:
     *
     * After partitioning around a pivot of 5,
     *
     *   a[i] = membs[indices[i]]
     *
     *       p         q         r         s
     * a:[.. 2 4 3 1 2 5 5 5 5 5 7 6 8 6 9 ..]
     *
     * /q/ comes in as an index whose corresponding
     * array element is the number 5.
     */
    void
partition_indexed_reals (uint* indices, uint* ret_q, uint* ret_r,
                         const real* membs, uint p, uint q, uint s)
{
    uint i, r, n;
    real x;
        /* /q/ must be valid, so the array cannot be empty.*/
    Claim( p < s );
    r = s-1;
    swap_uint (&indices[r], &indices[q]);
    x = membs[indices[r]];
    q = p;
    i = p;
    while (i < r)
    {
        real y;
        y = membs[indices[i]];
        if (y > x)
        {
            ++i;
        }
        else if (y < x)
        {
            swap_uint (&indices[q], &indices[i]);
            ++q;
            ++i;
        }
        else if (y == x)
        {
            --r;
            swap_uint (&indices[r], &indices[i]);
        }
    }
    Claim( p <= q && q <= r && r < s );
    n = s - r;
    UFor( i, n )
        swap_uint (&indices[q+i], &indices[r+i]);
    *ret_q = q;
    *ret_r = q + n;
}

static
    void
quicksort_indexed_reals (uint* indices, uint p, uint s, const real* membs)
{
#if 1
        /* Using insertion sort for low counts is slightly faster.*/
    if (p + 10 >= s)
    {
        sort_few_indexed_reals (s - p, &indices[p], membs);
    }
    else
#else
    if (p + 1 < s)
#endif
    {
        uint q, r;
        partition_indexed_reals (indices, &q, &r, membs,
                                 p, p+(s-p)/2, s);
        quicksort_indexed_reals (indices, p, q, membs);
        quicksort_indexed_reals (indices, r, s, membs);
    }
}

static
    void
bubblesort_indexed_reals (uint* indices, uint p, uint s, const real* membs)
{
    uint nmembs;
    uint i;
    nmembs = s - p;
    indices = &indices[p];
    UFor( i, nmembs )
    {
        uint j, ti;
        ti = indices[i];
        UFor( j, i )
        {
            uint tj;
            tj = membs[j];

            if (membs[tj] > membs[ti])
            {
                indices[i] = tj;
                indices[j] = ti;
                ti = tj;
                Claim( indices[i] != indices[j] );
            }
        }
    }
}

    void
sort_indexed_reals (uint* indices, uint p, uint s, const real* membs)
{
    const bool use_quicksort = true;
    uint i;
    Claim( p <= s );
    if (use_quicksort)
        quicksort_indexed_reals (indices, p, s, membs);
    else
        bubblesort_indexed_reals (indices, p, s, membs);

    for (i = p+1; i < s; ++i)
    {
        uint ti, tj;
        ti = indices[i-1];
        tj = indices[i];
        Claim( membs[ti] <= membs[tj] );
    }
}

static
    uint
median5_indexed_reals (uint* indices, const real* membs, uint p, uint s)
{
    uint q, i;
    if (p + 5 >= s)
    {
        uint n;
        real a[5];
        n = s - p;
        UFor( i,  n )  a[i] = membs[indices[p+i]];
        sort_few_reals (n, a);
        UFor( i, n )
            if (a[n/2] == membs[indices[p+i]])
                return p+i;
        Claim( 0 );
        return 0;
    }

    q = p;
    for (i = p; i + 5 < s; i += 5)
    {
        uint j;
        real a[5];

        UFor( j, 5 )  a[j] = membs[indices[i+j]];
        j = mid5_reals (a);
        swap_uint (&indices[q], &indices[i+j]);
        q += 1;
    }
    i = p + (q - p) / 2;
    select_indexed_reals (indices, membs, p, i, q);
    return i;
}

    void
select_indexed_reals (uint* indices, const real* membs,
                      uint p, uint i, uint s)
{
    Claim( p <= i && i < s );
    while (p < s)
    {
        uint q, r;
        q = median5_indexed_reals (indices, membs, p, s);
        partition_indexed_reals (indices, &q, &r, membs, p, q, s);

        if (i < q)  s = q;
        else        p = r;
    }
}

    bool
verify_select_indexed_reals (uint p, uint i, uint s,
                             const uint* indices, const real* membs)
{
    uint ith, n;
    uint nlo = 0, neq = 0, nhi = 0;
    real x;

    ith = i - p;
    n = s - p;
    x = membs[indices[i]];

    while (p < s)
    {
        real y;
        y = membs[indices[p]];
        if (y <  x)
        {
            if (p >= i)  return false;
            ++ nlo;
        }
        else if (y == x)
        {
            ++ neq;
        }
        else if (y >  x)
        {
            if (p <= i)  return false;
            ++ nhi;
        }
        else
        {
            return false;
        }
        ++ p;
    }

    Claim( n == nlo + neq + nhi );
    return (nlo <= ith && nhi <= (n - ith - 1));
}

    /* Returns the next unique real's index.*/
    uint
consecutive_indexed_reals (uint q, uint s, const uint* indices, const real* membs)
{
    real x;
    uint r;
    x = membs[indices[q]];
    r = q+1;
    while (r < s && x == membs[indices[r]])
        ++r;
    return r;
}

    /* For jumps[i] = j, set jumps[j] = i */
    void
invert_jump_table (uint n, uint* jumps)
{
    uint sweepi;
    UFor( sweepi, n )
    {
        uint i, j;
        i = sweepi;
        j = jumps[i];
        while (jumps[sweepi] < n)
        {
            uint i0;
            Claim( j < n );

            i0 = i;
            i = j;
            j = jumps[i];
            jumps[i] = i0 + n;
        }
    }

    UFor( sweepi, n )
        jumps[sweepi] -= n;
}

static bool
ordered_lexi_real (uint ndims, const real* a, const real* b)
{
    uint i;
    UFor( i, ndims )
    {
        if (a[i] < b[i])  return true;
        if (a[i] > b[i])  return false;
    }
    return true;
}

static void
sort_indexed_lexi_reals (uint* jumps, uint* indices, real* coords,
                         uint nmembs, uint ndims, const real* lexis)
{
    uint dim;

    Claim( minimal_unique (nmembs, indices) );

    jumps[0] = nmembs;

    UFor( dim, ndims )
    {
        uint q = 0;
        while (q < nmembs)
        {
            uint i, s;

            s = jumps[q];
            for (i = q; i < s; ++i)
                coords[indices[i]] = lexis[dim + ndims * indices[i]];
            sort_indexed_reals (indices, q, s, coords);

            while (q < s)
            {
                uint r;
                r = consecutive_indexed_reals (q, s, indices, coords);
                jumps[q] = r;
                q = r;
            }
            Claim( q == s );
        }
    }

    Claim( minimal_unique (nmembs, indices) );

    if (nmembs > 0)
    {
        uint i;
        UFor( i, nmembs-1 )
        {
            assert (ordered_lexi_real (ndims,
                                       &lexis[ndims * indices[i]],
                                       &lexis[ndims * indices[i+1]]));
        }
    }
}

    /**
     * Be sure to call shuffle_jump_table() afterwards if you want to
     * use /jumps/. Calculating it fully destroys /indices/.
     **/
    uint
condense_lexi_reals (uint* jumps, uint* indices, real* coords,
                     uint n, uint ndims, const real* lexis)
{
    uint i, q;

    UFor( i, n )  indices[i] = i;
    sort_indexed_lexi_reals (jumps, indices, coords, n, ndims, lexis);

    i = 0;
    q = 0;
    while (i < n)
    {
        uint r;
        r = jumps[i];
        Claim( i < r );

        jumps[i] = jumps[q];
        jumps[q] = q;
        swap_uint (&indices[q], &indices[i]);

        ++i;
        while (i < r)
        {
            AssertEqA( ndims, lexis, lexis,
                       ndims * indices[q],
                       ndims * indices[i] );
            jumps[i] = q;
            ++i;
        }
        ++q;
    }

    UFor( i, n )
        AssertEqA( ndims, lexis, lexis,
                   ndims * indices[i],
                   ndims * indices[jumps[i]] );

        /* Fixup indices so points in range don't move.*/
    UFor( i, q )
    {
        uint* e;
        e = &indices[i];
        while (*e < q && *e != i)
        {
            swap_uint (&jumps[*e], &jumps[i]);
            swap_uint (&indices[*e], e);
        }
    }

    invert_jump_table (q, jumps);

    Claim( minimal_unique (q, jumps) );

    for (i = q; i < n; ++i)
        AssertEqA( ndims, lexis, lexis,
                   ndims * indices[i],
                   ndims * indices[jumps[jumps[i]]] );

        /* Fixup duplicates' jumps to indices.*/
    for (i = q; i < n; ++i)
        jumps[i] = jumps[jumps[i]];

        /* And make the uniques' jumps consistent.*/

    UFor( i, q )
        jumps[i] = i;

    UFor( i, n )
        AssertEqA( ndims, lexis, lexis,
                   ndims * indices[i],
                   ndims * indices[jumps[i]] );

    Claim( minimal_unique (n, indices) );

    return q;
}

    /** Finish calculation of /jumps/.
     * Both /jumps/ and /indices/ come in with valid data.
     * When finished, /indices/ is thrashed.
     **/
    void
shuffle_jump_table (uint n, uint* jumps, uint* indices)
{
    uint i;

    UFor( i, n )
    {
        uint pi;
        pi = indices[i];
        indices[i] = n;  /* Never get info from this location again!*/
        while (pi != i)
        {
            Claim( i < pi );
            swap_uint (&jumps[i], &jumps[pi]);
            swap_uint (&pi, &indices[pi]);
        }
    }
}

