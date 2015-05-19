
#include "order.h"

#include <assert.h>

static void
testfn_partition ()
{
    uint indices[] = { 0, 1, 2, 3, 4, 5 };
    const real membs[] = { 10, 112, 333, 254, 305, 406 };
    uint q, r;
    partition_indexed_reals (indices, &q, &r, membs, 0, 4, 6);
    assert (indices[3] == 4);
}

static void
testfn_select ()
{
    uint i;
    const uint nmembs = 7;
    uint indices[7];
    const uint ordered[] = { 0, 6, 1, 3, 4, 2, 5 };
    const real membs[] = { 10, 112, 333, 254, 305, 406, 19 };

    UFor( i, nmembs )
    {
        uint j;
        UFor( j, nmembs )  indices[j] = j;
        select_indexed_reals (indices, membs, 0, i, nmembs);
        AssertEq( ordered[i], indices[i] );
    }
}

static void
testfn_condense_lexi_reals ()
{
#define N 11
    real lexis[2 * N] =
    {    0, 0 /*0*/
        ,1, 1
        ,2, 1
        ,3, 3
        ,3, 3
        ,2, 2 /*5*/
        ,2, 2
        ,1, 1
        ,1, 1
        ,1, 1
        ,0, 0
    };
    const uint expect_jumps[] =
    { 0, 1, 2, 3, 3, 4, 4, 1, 1, 1, 0 };
    uint jumps[N];
    uint indices[N];
    real coords[N];
    uint i, n;

    AssertEq( N, ArraySz( expect_jumps ) );

    n = condense_lexi_reals (jumps, indices, coords,
                             N, 2, lexis);
    AssertEq( 5, n );  /* This is 1 + max in expect_jumps.*/
    UFor( i, n )
        AssertEq( i, expect_jumps[indices[i]] );

    shuffle_jump_table (N, jumps, indices);
    UFor( i, N )
        AssertEq( expect_jumps[i], jumps[i] );
#undef N
}

    void
testfn_order ()
{
    testfn_partition ();
    testfn_select ();
    testfn_condense_lexi_reals ();
}

