
#ifndef Color_H_
#ifndef __OPENCL_VERSION__
#define Color_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */
#undef NDims
#define NDims NColors
#define Point Color
#include "point.h"
#undef NDims
#define NDims NDimensions
#undef Point
#endif

