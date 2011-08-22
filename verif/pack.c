
#include "pack.h"

#include <assert.h>

static
    void
testfn_realPack ()
{
    union test_union
    {
        real v[realPackSz];
        realPack x;
    } u;
    realPack x;
    real v[realPackSz];
    uint i;

    UFor( i, realPackSz )
        u.v[i] = v[i] = .5 * i;

    x = u.x;


    u.x = summ_realPack (x, x);
    UFor( i, realPackSz )
        assert (u.v[i] == v[i] + v[i]);

    u.x = diff_realPack (x, fill_realPack (7));
    UFor( i, realPackSz )
        assert (u.v[i] == v[i] - 7);

    u.x = mult_realPack (x, fill_realPack (5));
    UFor( i, realPackSz )
        assert (u.v[i] == v[i] * 5);

    u.x = divi_realPack (x, fill_realPack (2));
    UFor( i, realPackSz )
        assert (u.v[i] == v[i] / 2);

    u.x = abs_realPack (mult_realPack (x, fill_realPack (-3)));
    UFor( i, realPackSz )
        assert (u.v[i] == abs_real (v[i] * -3));

    u.x = zero_realPack ();
    UFor( i, realPackSz )
        assert (u.v[i] == 0);
}

    void
testfn_pack ()
{
    testfn_realPack ();
}

