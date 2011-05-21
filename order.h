
#ifndef ORDER_H_
#define ORDER_H_
#include "util.h"

bool
minimal_unique (uint n, const uint* a);
void
sort_few_uints (uint n, uint* a);
void
sort_indexed_reals (uint nmembs, uint* indices, const real* coords);

#ifdef INCLUDE_SOURCE
#include "order.c"
#endif
#endif

