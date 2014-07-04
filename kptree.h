
#ifndef KPTREE_H_
#ifndef __OPENCL_VERSION__
#define KPTREE_H_

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
    BBox box;
};


void
set1_KPTreeGrid (KPTreeGrid* grid, uint i, const Point* p);
uint
descend_KPTree (const KPTree* tree, const Point* loc, uint i);
uint
next_KPTree (const KPTree* tree, const Point* loc,
             uint* ret_i, real* ret_mag2);
uint
nearest_neighbor_KPTree (const KPTree* tree, const Point* loc);
uint
inside_BBox_KPTree (const KPTree* tree, const BBox* box, uint i);
#ifndef __OPENCL_VERSION__
void
init_KPTree (KPTree* tree);
void
lose_KPTree (KPTree* tree);
void
init_KPTreeGrid (KPTreeGrid* grid, uint n);
void
lose_KPTreeGrid (KPTreeGrid* grid);
void
build_KPTree (KPTree* tree, KPTreeGrid* grid);

#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

