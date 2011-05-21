
#include "bitstring.h"
#include "util.h"

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

int main ()
{
    testfn_BitString ();
    return 0;
}

