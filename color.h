
#ifndef Color_H_
#define Color_H_
#include "space.h"
#undef NDims
#define NDims NColors
#define Point Color
#include "point.h"
#undef NDims
#define NDims NDimensions
#endif

