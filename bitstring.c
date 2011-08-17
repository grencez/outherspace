
#include "bitstring.h"

#include <string.h>

    BitString*
alloc_BitString (uint n, bool val)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    uint nblocks, nbytes;
    bitstring_t* bs;

    nblocks = ceil_uint (n, nblockbits);
    nbytes = nblocks * sizeof (bitstring_t);

    bs = AllocT( bitstring_t, nblocks );

    if (bs)
    {
        if (val)  memset (bs, 0xFF, nbytes);
        else      memset (bs, 0x00, nbytes);
    }
    return bs;
}

    void
free_BitString (BitString* bs)
{
    free (bs);
}

    void
zero_BitString (BitString* bs, uint n)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    uint i;
    n = ceil_uint (n, nblockbits);
    UFor( i, n )  bs[i] = 0;
}

    bool
test_BitString (const BitString* bs, uint i)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    uint p, q;
    p = i / nblockbits;
    q = i % nblockbits;
    return 0 != (bs[p] & (1 << q));
}

    bool
set1_BitString (BitString* bs, uint i)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    uint p, q;
    bitstring_t x, y;

    p = i / nblockbits;
    q = i % nblockbits;
    x = bs[p];
    y = 1 << q;

    if (0 != (x & y))
    {
        return true;
    }
    else
    {
        bs[p] = x | y;
        return false;
    }
}

    bool
set0_BitString (BitString* bs, uint i)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    uint p, q;
    bitstring_t x, y;

    p = i / nblockbits;
    q = i % nblockbits;
    x = bs[p];
    y = 1 << q;

    if (0 == (x & y))
    {
        return false;
    }
    else
    {
        bs[p] = x & ~y;
        return true;
    }
}

    bool
all_BitString (uint n, const BitString* bs)
{
    const uint nblockbits = sizeof (bitstring_t) * NBitsInByte;
    const bitstring_t all_true = ~ (bitstring_t) 0;
    uint p, q;
    uint i;

    p = n / nblockbits;
    q = n % nblockbits;

    UFor( i, p )
        if (bs[i] != all_true)  return false;

    UFor( i, q )
        if (!test_BitString (bs, n - 1 - i))  return false;

    return true;
}

