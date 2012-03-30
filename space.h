
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
typedef struct Point2 Point2;
typedef struct Point3 Point3;
typedef struct Color Color;
typedef struct Ray Ray;

struct Point
{
    real coords[NDims];
};

struct Point2 { real coords[2]; };
struct Point3 { real coords[3]; };

#define NColors 3
struct Color { real coords[3]; };

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

typedef struct BBox BBox;

struct BBox
{
    Point min;
    Point max;
};

typedef PointXfrm Simplex;
typedef struct Plane Plane;
typedef struct BaryPoint BaryPoint;
typedef struct BarySimplex BarySimplex;

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

