
#ifndef ORDER_H_
#define ORDER_H_
#include "util.h"

void
fill_minimal_unique (uint* a, uint n);
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
void
invert_jump_table (uint n, uint* jumps);
uint
condense_lexi_reals (uint* jumps, uint* indices, real* coords,
                     uint n, uint ndims, const real* lexis);
void
shuffle_jump_table (uint n, uint* jumps, uint* indices);
#endif

