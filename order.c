
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

    assert (0);
    return 0;
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
        assert (a[i] < n);
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
    assert (p < s);
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
    assert (p <= q && q <= r && r < s);
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
    if (p + 1 < s)
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
bubblesort_indexed_reals (uint nmembs, uint* indices, const real* membs)
{
    uint i;
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
                assert (indices[i] != indices[j]);
            }
        }
    }
}

    void
sort_indexed_reals (uint nmembs, uint* indices, const real* membs)
{
    uint i;
    const bool use_quicksort = true;
    assert (even_uint (nmembs));
    assert (minimal_unique (nmembs, indices));
    if (use_quicksort)
        quicksort_indexed_reals (indices, 0, nmembs, membs);
    else
        bubblesort_indexed_reals (nmembs, indices, membs);
    assert (minimal_unique (nmembs, indices));
    if (nmembs > 0)
    {
        UFor( i, nmembs-1 )
        {
            uint ti, tj;
            ti = indices[i];
            tj = indices[i+1];
            assert (membs[ti] <= membs[tj]);
        }
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
        assert (0);
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
    assert (p <= i && i < s);
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

    assert (n == nlo + neq + nhi);
    return (nlo <= ith && nhi <= (n - ith - 1));
}

