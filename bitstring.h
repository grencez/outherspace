

#ifndef BITSTRING_H_
#define BITSTRING_H_

#include "util.h"

typedef size_t bitstring_t;
typedef bitstring_t BitString;

BitString*
alloc_BitString (uint n, bool val);
void
free_BitString (BitString* bs);
bool
test_BitString (BitString* bs, uint i);
bool
set1_BitString (BitString* bs, uint i);

#ifdef INCLUDE_SOURCE
#include "bitstring.c"
#endif
#endif

