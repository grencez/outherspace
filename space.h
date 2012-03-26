
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

#if 0
#define FoDim 0
#define UpDim 1
#define RiDim 2
#else
#define UpDim 0
#define RiDim 1
#define FoDim 2
#endif
#define ForwardDim FoDim
#define RightDim RiDim

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

