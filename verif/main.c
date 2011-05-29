
#include "bitstring.h"
#include "util.h"
#include "xfrm.h"

#include <assert.h>

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
}

    /* This mimics the dirty bit in a set associative cache,
     * but is unrealistic since it disregards any values.
     * Now, if all values fall inside [0..255], then we have a useful tool,
     * but then LowBits() would not be tested.
     */
    void
testfn_BitString_cache ()
{
    uint i;
    bool flag;
    Declare_BitString( cache, 256 );
    const uint nslots = 256;
    const uint nbits = 8;

    set1_BitString (cache, 100);
    zero_BitString (cache, 256);
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

int main ()
{
    testfn_BitString ();
    testfn_BitString_cache ();
    testfn_PointXfrm ();
    return 0;
}

