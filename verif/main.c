
#include "affine.h"
#include "bbox.h"
#include "bitstring.h"
#include "gmrand.h"
#include "kdtree.h"
#include "kptree.h"
#include "order.h"
#include "point.h"
#include "scene.h"
#include "serial.h"
#include "slist.h"
#include "util.h"
#include "xfrm.h"

#include <assert.h>
#include <math.h>

void testfn_order ();
void testfn_pack ();

static void
testfn_IAMap ()
{
    IAMap map;
    Point u, v;

    zero_Point (&u);
    u.coords[0] =  1;
    u.coords[1] = -2;
    identity_IAMap (&map);
    scale0_IAMap (&map, 3);
    xlat0_IAMap (&map, &u);

    zero_Point (&v);
    map_Point (&v, &map, &v);
    AssertApprox(  3, v.coords[0], 1, 1e0 );
    AssertApprox( -6, v.coords[1], 1, 1e0 );

    identity_IAMap (&map);
    scale0_IAMap (&map, .5);
    map_Point (&v, &map, &v);
    AssertApprox(  1.5, v.coords[0], 1, 1e0 );
    AssertApprox( -3.0, v.coords[1], 1, 1e0 );
}

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
    BBox box;
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


    zero_BBox (&box);
    UFor( i, 2 )
    {
        box.min.coords[i] = 1.5;
        box.max.coords[i] = 4.1;
    }

    i = inside_BBox_KPTree (&tree, &box, Max_uint);
    while (i != Max_uint)
    {
        FILE* out = stderr;
        output_Point (out, &tree.nodes[i].loc);
        fputc ('\n', out);
        i = inside_BBox_KPTree (&tree, &box, i);
    }

    cleanup_KPTree (&tree);
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
testfn_trxfrm_BBox ()
{
    BBox box;
    Point centroid;
    PointXfrm basis;

    zero_BBox (&box);
    box.min.coords[0] = 2;
    box.max.coords[0] = 5;

    zero_Point (&centroid);

    rotation_PointXfrm (&basis, 0, 1,  M_PI / 2);
    transpose_PointXfrm (&basis, &basis);

    trxfrm_BBox (&box, &basis, &box, &centroid);
    AssertApprox( 0, box.min.coords[0], 2, 1e0 );
    AssertApprox( 2, box.min.coords[1], 2, 1e0 );
    AssertApprox( 0, box.max.coords[0], 5, 1e0 );
    AssertApprox( 5, box.max.coords[1], 5, 1e0 );
}

static
    void
testfn_serial ()
{
    GMRand gmrand;
    init_GMRand (&gmrand);

    { BLoop( testidx, 1e5 )
        DecloStack( FileB, f );
        DecloStack( FileB, olay );
        Point expect, result;

        { BLoop( i, NDims )
            expect.coords[i] = 1e7 * real_GMRand (&gmrand);
        } BLose()

        init_FileB (f);
        f->sink = true;
        dumpp_Point (f, &expect);
        olay_FileB (olay, f);
        load_Point (olay, &result);

        Claim( equal_Point (&expect, &result) );
        lose_FileB (olay);
        lose_FileB (f);
    } BLose()
}

    /** First crack at a verification function.
     * This is not a truly good one because it uses random values
     * instead of iterating through all possible ones.
     **/
static
    void
verifn_orthorotate_PointXfrm (uint npids, uint pidx)
{
    GMRand gmrand;
    uint i, n;
    n = 1 << 20;

    init_GMRand (&gmrand);
    step_GMRand (&gmrand, pidx);

    for (i = pidx; i < n; i += npids)
    {
        PointXfrm A;
        Point v;
        uint j;
        real det;

        identity_PointXfrm (&A);

        UFor( j, NDimensions )
        {
            v.coords[j] = real_GMRand (&gmrand);
            step_GMRand (&gmrand, npids-1);
            if (bool_GMRand (&gmrand))
                v.coords[j] = - v.coords[j];
            step_GMRand (&gmrand, npids-1);
        }

        stable_orthorotate_PointXfrm (&A, &A, &v,
                                      uint_GMRand (&gmrand, NDimensions));
        step_GMRand (&gmrand, npids-1);
        det = det_PointXfrm (&A);
            /* fprintf (stderr, "det:%.32f\n", det); */
        AssertApprox( 1, det, NDimensions, 2e0 );
    }
}

static
    void
testfn_SList ()
{
    const uint counts[]  = { 19,  1, 0, 13,  5 };
    const uint factors[] = { 3,   5, 7, 11, 17 };
    SList a, b;
    uint i, n;
    uint* membs;

    init_SList (&a);  init_SList (&b);

    n = 0;
    UFor( i, 5 )
    {
        uint j;

        UFor( j, counts[i] )
        {
            if (even_uint (i))  app_uint_SList (&b, factors[i] * j);
            else                app_uint_SList (&a, factors[i] * j);
        }

        cat_SList (&a, &b);

        assert (a.nmembs == n + counts[i]);
        assert (b.nmembs == 0);

        if (counts[i] > 0)
        {
            j = *(uint*) aref_SList (&a, n);
            assert (j == 0);
        }
        if (counts[i] > 1)
        {
            j = *(uint*) aref_SList (&a, n + counts[i] - 1);
            assert (j == factors[i] * (counts[i] - 1));
        }
        n += counts[i];
    }

    membs = AllocT( uint, n );
    unroll_SList (membs, &a, sizeof(uint));

    UFor( i, n )
    {
        uint x, j, k;
        x = membs[i];
        j = i;
        k = 0;
        while (j >= counts[k])
        {
            j -= counts[k];
            k += 1;
            assert (k < 5);
        }
        assert (x == j * factors[k]);
    }
    free (membs);
}

int main (int argc, char** argv)
{
    int argidx = 1;
    uint npids = 1, pidx = 0;
    if (argidx < argc)
        strto_uint (&npids, argv[argidx++]);

    if (argidx < argc)
        strto_uint (&pidx, argv[argidx++]);

    testfn_IAMap ();
    testfn_BitString ();
    testfn_BitString_cache ();
    testfn_KPTree ();
    testfn_PointXfrm ();
    testfn_trxfrm_BBox ();
    testfn_serial ();
    testfn_SList ();
    testfn_order ();
    testfn_pack ();
    verifn_orthorotate_PointXfrm (npids, pidx);
    return 0;
}

