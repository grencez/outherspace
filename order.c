
#include "order.h"
#include "bitstring.h"

#include <assert.h>

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


static
    void
swap_uint (uint* x, uint* y)
{
    uint tmp = *x;
    *x = *y;
    *y = tmp;
}


static
    void
partition_indexed_reals (uint* indices, uint* ret_q, uint* ret_r,
                         uint p, uint s, const real* membs)
{
    uint i, q, r, n;
    real x;
    assert (p < s);
    swap_uint (&indices[s], &indices[p+(s-p)/2]);
    x = membs[indices[s]];
    q = p;
    r = s;
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
    assert (p <= q && q <= r && r <= s);
    n = s - r + 1;
    UFor( i, n )
        swap_uint (&indices[q+i], &indices[r+i]);
    *ret_q = q;
    *ret_r = q + n;
}

static
    void
quicksort_indexed_reals (uint* indices, uint p, uint s, const real* membs)
{
    if (p < s)
    {
        uint q, r;
        partition_indexed_reals (indices, &q, &r, p, s, membs);
        if (q > 0)
            quicksort_indexed_reals (indices, p, q -1, membs);
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
        quicksort_indexed_reals (indices, 0, nmembs-1, membs);
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

