
#ifndef Op_Point_0

#include "pack.h"

#define Op_Point_0( dst, a ) \
    Op_0( real, NDimensions, (dst)->coords, (a)->coords )

#define Op_Point_10( dst, f, a ) \
    Op_10( real, NDimensions, (dst)->coords, f, (a)->coords )

#define Op_Point_200( dst, f, a, b ) \
    Op_200( real, NDimensions, (dst)->coords, f, (a)->coords, (b)->coords )

#define Op_Point_1200( dst, f, g, a, b ) \
    Op_1200( real, NDimensions, (dst)->coords, \
             f, g, (a)->coords, (b)->coords )

#define Op_Point_2010( dst, f, a, g, b ) \
    Op_2010( real, NDimensions, (dst)->coords, \
             f, (a)->coords, g, (b)->coords )

#define Op_Point_2100( dst, f, g, a, b ) \
    Op_2100( real, NDimensions, (dst)->coords, \
             f, g, (a)->coords, (b)->coords )

#define Op_Point_21010( dst, f, g, a, h, b ) \
    Op_21010( real, NDimensions, (dst)->coords, \
              f, g, (a)->coords, h, (b)->coords )

#define Op_Point_201200( dst, f1, a, f2, f3, b, c ) \
    Op_201200( real, NDimensions, (dst)->coords, \
               f1, (a)->coords, f2, f3, (b)->coords, (c)->coords )

#define Op_Point_202100( dst, f, a, g, h, b, c ) \
    Op_202100( real, NDimensions, (dst)->coords, \
               f, (a)->coords, g, h, (b)->coords, (c)->coords )

#define Op_Point_2021010( dst, f1, a, f2, f3, b, f4, c ) \
    Op_2021010( real, NDimensions, (dst)->coords, f1, \
                (a)->coords, f2, f3, (b)->coords, f4, (c)->coords )

#endif

