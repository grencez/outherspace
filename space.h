
#ifndef SPACE_H_
#define SPACE_H_
#include "cx/def.h"

#ifndef NDimensions
#define NDimensions 3
#endif

#ifdef NDims
#undef NDims
#endif
#define NDims NDimensions

typedef struct Point Point;
typedef struct Ray Ray;

struct Point
{
    real coords[NDims];
};

struct Ray
{
    Point origin;
    Point direct;
};

    /* #define ForwardDim (NDims - 1) */
#define UpDim 0
#define RightDim 1
#define ForwardDim 2
#define FwdDim ForwardDim

typedef struct PointXfrm PointXfrm;

struct PointXfrm
{
    Point pts[NDims];
};

typedef struct BoundingBox BoundingBox;

struct BoundingBox
{
    Point min;
    Point max;
};

typedef struct Simplex Simplex;
typedef struct Plane Plane;
typedef struct BaryPoint BaryPoint;
typedef struct BarySimplex BarySimplex;

struct Simplex
{
    Point pts[NDims];
};

struct Plane
{
    real offset;
    Point normal;
};

struct BaryPoint
{
    real coords[NDims-1];
};

struct BarySimplex
{
    Plane plane;
    Plane barys[NDims-1];
};

#endif

