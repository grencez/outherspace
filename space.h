
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

#if 0
#define FwDim 0
#define UpDim 1
#define RtDim 2
#else
#define UpDim 0
#define RtDim 1
#define FwDim 2
#endif
#define ForwardDim FwDim
#define RightDim RtDim

typedef struct Point Point;
typedef struct BaryPoint BaryPoint;
typedef struct Point2 Point2;
typedef struct Point3 Point3;
typedef struct Color Color;
typedef struct Ray Ray;
typedef struct PointXfrm PointXfrm;
typedef struct IAMap IAMap;
typedef struct BBox BBox;
typedef PointXfrm Simplex;
typedef struct Plane Plane;
typedef struct BarySimplex BarySimplex;


struct Point { real coords[NDims]; };
struct BaryPoint { real coords[NDims-1]; };
struct Point2 { real coords[2]; };
struct Point3 { real coords[3]; };

#define NColors 3
struct Color { real coords[3]; };

struct Ray
{
    Point origin;
    Point direct;
};

struct PointXfrm { Point pts[NDims]; };

    /**
     * map (x) = xlat + xfrm scale x
     * map^-1 (x) = (xfrm^-1 (x - xlat)) / scale
     **/
struct IAMap
{
    PointXfrm xfrm;
    Point xlat;
    Point scale;
};


struct BBox
{
    Point min;
    Point max;
};

struct Plane
{
    real offset;
    Point normal;
};

struct BarySimplex
{
    Plane plane;
    Plane barys[NDims-1];
};

#endif

