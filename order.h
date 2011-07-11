
#ifndef ORDER_H_
#define ORDER_H_
#include "util.h"

bool
minimal_unique (uint n, const uint* a);
void
sort_few_uints (uint n, uint* a);
void
sort_few_reals (uint n, real* a);
void
partition_indexed_reals (uint* indices, uint* ret_q, uint* ret_r,
                         const real* membs, uint p, uint q, uint s);
void
sort_indexed_reals (uint* indices, uint p, uint s, const real* membs);
void
select_indexed_reals (uint* indices, const real* membs,
                      uint p, uint i, uint s);
bool
verify_select_indexed_reals (uint p, uint i, uint s,
                             const uint* indices, const real* membs);
uint
consecutive_indexed_reals (uint q, uint s,
                           const uint* indices,
                           const real* membs);
#ifdef INCLUDE_SOURCE
#include "order.c"
#endif
#endif

