
#ifndef KPTREE_H_
#ifndef __OPENCL_VERSION__
#define KPTREE_H_

#include "bbox.h"
#include "simplex.h"
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct KPTreeNode KPTreeNode;
typedef struct KPTree KPTree;
typedef struct KPTreeGrid KPTreeGrid;

struct KPTreeNode
{
    uint dim;
    uint idx;
    Point loc;
};

struct KPTree
{
    uint nnodes;
    KPTreeNode* nodes;
};

struct KPTreeGrid
{
    uint npts;
    uint* indices;
    real* coords[NDimensions];
    BoundingBox box;
};


uint
descend_KPTree (const KPTree* tree, const Point* loc, uint i);
uint
nearest_neighbor_KPTree (const KPTree* tree, const Point* loc);
uint
inside_BoundingBox_KPTree (const KPTree* tree,
                           const BoundingBox* box,
                           uint i);
#ifndef __OPENCL_VERSION__
void
init_KPTree (KPTree* tree);
void
cleanup_KPTree (KPTree* tree);
void
cleanup_KPTreeGrid (KPTreeGrid* grid);
void
build_KPTree (KPTree* tree, KPTreeGrid* grid);

#ifdef IncludeC
#include "kptree.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

