
#include "cx/syscx.h"
#include "cx/fileb.h"
#include "cx/bittable.h"
#include "cx/urandom.h"
#include "affine.h"
#include "bbox.h"
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

    lose_KPTree (&tree);
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
  URandom urandom;
  init_URandom (&urandom);

  {:for (testidx ; 1e5)
    OFile of[] = default;
    XFile olay[1];
    Point expect, result;

    {:for (i ; NDims)
      expect.coords[i] = 1e7 * real_URandom (&urandom);
    }

    oput_Point (of, &expect);
    olay_txt_OFile (olay, of, 0);
    xget_Point (olay, &result);

    Claim( equal_Point (&expect, &result) );
    lose_XFile (olay);
    lose_OFile (of);
  }
}

    /** First crack at a verification function.
     * This is not a truly good one because it uses random values
     * instead of iterating through all possible ones.
     **/
static
    void
verifn_orthorotate_PointXfrm (uint npids, uint pidx)
{
    URandom urandom;
    uint i, n;
    n = 1 << 20;

    init2_URandom (&urandom, pidx, npids);

    for (i = pidx; i < n; i += npids)
    {
        PointXfrm A;
        Point v;
        uint j;
        real det;

        identity_PointXfrm (&A);

        UFor( j, NDimensions )
        {
            v.coords[j] = real_URandom (&urandom);
            if (bool_URandom (&urandom))
                v.coords[j] = - v.coords[j];
        }

        stable_orthorotate_PointXfrm (&A, &A, &v,
                                      uint_URandom (&urandom, NDimensions));
        det = det_PointXfrm (&A);
            /* fprintf (stderr, "det:%.32f\n", det); */
        AssertApprox( 1, det, NDimensions, 5e0 );
    }
}

static void
testfn_orthorotate_PointXfrm ()
{
  verifn_orthorotate_PointXfrm (1, 0);
}

static
    void
testfn_SList ()
{
    const uint counts[]  = { 19,  1, 0, 13,  5 };
    const uint factors[] = { 3,   5, 7, 11, 17 };
    SList a = default;
    SList b = default;
    uint i, n;
    uint* membs;

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

    AllocTo( membs, n );
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


static
  void
Test(const char testname[])
{
  void (*fn) () = 0;

  /* cswitch testname
   *   -case-pfx "fn = testfn_"
   *   -array AllTests
   *   -x testlist.txt
   *   -o testswitch.c
   */
#include "testswitch.c"

  if (fn) {
    fn();
  }
  else if (!testname[0]) {
    for (uint i = 0; i < ArraySz(AllTests); ++i) {
      Test(AllTests[i]);
    }
  }
  else {
    Claim( 0 && "Test does not exist." );
  }
}


int main(int argc, char** argv)
{
  int argi = (init_sysCx (&argc, &argv), 1);

#if 0
  uint npids = 1, pidx = 0;
  if (argi < argc)
    strto_uint (&npids, argv[argi++]);

  if (argi < argc)
    strto_uint (&pidx, argv[argi++]);
#endif

  if (argi == argc) {
    Test("");
  }
  else {
    while (argi < argc) {
      Test(argv[argi++]);
    }
  }

  lose_sysCx ();
  return 0;
}

