
#include "bitstring.h"
#include "kdtree.h"
#include "kptree.h"
#include "order.h"
#include "util.h"
#include "xfrm.h"

#include <assert.h>

static
    void
testfn_BitString ()
{
    uint i, n, ni;
    BitString* bs;

    n = 1000;
    bs = alloc_BitString (n, 0);

    ni = ceil_uint (n, 3);
    UFor( i, ni )
    {
        bool x;
        x = set1_BitString (bs, 3 * i);
        assert (!x);
    }

    UFor( i, n )
    {
        bool x, y;
        x = test_BitString (bs, i);
        y = (0 == (i % 3));
        assert (x == y);
        x = set1_BitString (bs, i);
        assert (x == y);
    }
    free (bs);
}

    /* This mimics the dirty bit in a set associative cache,
     * but is unrealistic since it disregards any values.
     * Now, if all values fall inside [0..255], then we have a useful tool,
     * but then LowBits() would not be tested.
     */
static
    void
testfn_BitString_cache ()
{
    uint i;
    bool flag;
    Declare_BitString( cache, 256 );
    const uint nslots = 256;
    const uint nbits = 8;

    zero_BitString (cache, nslots);
    set1_BitString (cache, 100);
    zero_BitString (cache, nslots);
    UFor( i, nslots )
        assert (!test_BitString (cache, i));

    i = LowBits( uint, nbits, nslots+1 );
    flag = set1_BitString (cache, i);
    assert (i == 1 && !flag);
    i = LowBits( uint, nbits, nslots-1 );
    flag = set1_BitString (cache, i);
    assert (i == nslots-1 && !flag);
    i = LowBits( uint, nbits, 3*(nslots-1) );
    flag = set1_BitString (cache, i);
    assert (i == nslots-3 && !flag);
    i = LowBits( uint, nbits, 5*nslots-3 );
    flag = set1_BitString (cache, i);
    assert (i == nslots-3 && flag);
}

static
    void
testfn_KPTree ()
{
    uint i;
    KPTree tree;
    KPTreeGrid grid;
    BoundingBox box;
    uint indices[6] = { 0, 1, 2, 3, 4, 5 };
    real coords[3][6] =
    {
        { 0, 1, 2, 3, 4, 5 },
        { 0, 1, 2, 3, 4, 5 },
        { 0, 0, 0, 0, 0, 0 }
    };
    Point loc;

    grid.npts = 6;

    grid.indices = indices;

    grid.coords[0] = coords[0];
    grid.coords[1] = coords[1];
    UFor( i, NDimensions-2 )
        grid.coords[2+i] = coords[2];  /* Zero'd out.*/
    zero_Point (&grid.box.min);
    zero_Point (&grid.box.max);
    grid.box.max.coords[0] = grid.npts-1;
    grid.box.max.coords[1] = grid.npts-1;

    build_KPTree (&tree, &grid);

    zero_Point (&loc);
    i = nearest_neighbor_KPTree (&tree, &loc);
    i = tree.nodes[i].idx;
    assert (i == 0);

    loc.coords[0] = 1;
    loc.coords[1] = 1.9;
    i = nearest_neighbor_KPTree (&tree, &loc);
    i = tree.nodes[i].idx;
    assert (i == 1);

    loc.coords[0] = 1;
    loc.coords[1] = 2.1;
    i = nearest_neighbor_KPTree (&tree, &loc);
    i = tree.nodes[i].idx;
    assert (i == 2);

    loc.coords[0] = 2;
    loc.coords[1] = 6;
    i = nearest_neighbor_KPTree (&tree, &loc);
    i = tree.nodes[i].idx;
    assert (i == 4);

    loc.coords[0] = 6;
    loc.coords[1] = 2;
    i = nearest_neighbor_KPTree (&tree, &loc);
    i = tree.nodes[i].idx;
    assert (i == 4);


    zero_BoundingBox (&box);
    UFor( i, 2 )
    {
        box.min.coords[i] = 1.5;
        box.max.coords[i] = 4.1;
    }

    i = inside_BoundingBox_KPTree (&tree, &box, Max_uint);
    while (i != Max_uint)
    {
        FILE* out = stderr;
        output_Point (out, &tree.nodes[i].loc);
        fputc ('\n', out);
        i = inside_BoundingBox_KPTree (&tree, &box, i);
    }
}

static
    void
testfn_partition ()
{
    uint indices[] = { 0, 1, 2, 3, 4, 5 };
    const real membs[] = { 10, 112, 333, 254, 305, 406 };
    uint q, r;
    partition_indexed_reals (indices, &q, &r, membs, 0, 4, 6);
    assert (indices[3] == 4);
}

static
    void
testfn_PointXfrm ()
{
    uint i, j;
    real det;
    PointXfrm A, B, C;
    Point u, v, w;

        /* /A/ and /B/ are inverse of each other.*/
    rotation_PointXfrm (&A, 0, 1, M_PI / 3);
    rotation_PointXfrm (&B, 1, 0, M_PI / 3);

    zero_Point (&u);
    u.coords[0] = 1;
    xfrm_Point (&v, &A, &u);
    xfrm_Point (&w, &A, &v);
    xfrm_Point (&u, &A, &w);

    AssertApprox(  .5,            v.coords[0], 1, 1e0 );
    AssertApprox(  .5 * sqrt (3), v.coords[1], 1, 1e0 );
    AssertApprox( -.5,            w.coords[0], 1, 1e0 );
    AssertApprox(  .5 * sqrt (3), w.coords[1], 1, 1e0 );
    AssertApprox(  -1,            u.coords[0], 1, 1e0 );
    AssertApprox(   0,            u.coords[1], 1, 2e0 );

        /* Inverse of a rotation matrix is equal to its transpose.*/
    transpose_PointXfrm (&C, &B);
    UUFor( i, NDimensions, j, NDimensions )
        assert (A.pts[i].coords[j] == C.pts[i].coords[j]);

        /* Determinant of a rotation matrix is 1.*/
    det = det2_PointXfrm (&A, 0, 1, 0, 1);
    AssertApprox( 1, det, 1, 1e0 );
    det = det3_PointXfrm (&A, 0, 1, 2, 0, 1, 2);
    AssertApprox( 1, det, 1, 1e0 );
    det = det_PointXfrm (&A);
    AssertApprox( 1, det, 1, 1e0 );

        /* Multiplying by an inverse gives the identity.*/
    xfrm_PointXfrm (&C, &A, &B);
    identity_PointXfrm (&B);
    UUFor( i, NDimensions, j, NDimensions )
        AssertApprox( B.pts[i].coords[j], C.pts[i].coords[j], 1, 1e0 );

        /* If we scale a row by 2, and another by 3,
         * the determinant should be 6.
         */
    copy_PointXfrm (&C, &A);
        /* Use /A/'s row to assure the above copy worked.*/
    scale_Point (&C.pts[0], &A.pts[0], 2);
    scale_Point (&C.pts[1], &C.pts[1], 3);
    det = det2_PointXfrm (&C, 0, 1, 0, 1);
    AssertApprox( 6, det, 6, 1e0 );
    det = det3_PointXfrm (&C, 0, 1, 2, 0, 1, 2);
    AssertApprox( 6, det, 6, 1e0 );
    det = det_PointXfrm (&C);
    AssertApprox( 6, det, 6, 1e0 );
}

static
    void
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
        assert (indices[i] == ordered[i]);
    }
}

int main ()
{
    testfn_BitString ();
    testfn_BitString_cache ();
    testfn_KPTree ();
    testfn_partition ();
    testfn_PointXfrm ();
    testfn_select ();
    return 0;
}

