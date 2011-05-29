

#ifndef BITSTRING_H_
#define BITSTRING_H_

#include "util.h"

typedef unsigned int bitstring_t;
typedef bitstring_t BitString;

#define Declare_BitString( bs, n ) \
    bitstring_t bs[Ceil_uint(n,NBitsInByte*sizeof(bitstring_t))]

#define LowBits( Type, nbits, val ) \
    (val) & (~((~(Type)0) << (nbits)))

BitString*
alloc_BitString (uint n, bool val);
void
free_BitString (BitString* bs);
void
zero_BitString (BitString* bs, uint n);
bool
test_BitString (BitString* bs, uint i);
bool
set1_BitString (BitString* bs, uint i);

#ifdef INCLUDE_SOURCE
#include "bitstring.c"
#endif
#endif

