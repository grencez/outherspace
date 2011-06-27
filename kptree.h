
#ifndef KPTREE_H_
#ifndef __OPENCL_VERSION__
#define KPTREE_H_

#include "simplex.h"
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct kp_tree_node_struct;
struct kp_tree_struct;
typedef struct kp_tree_node_struct KPTreeNode;
typedef struct kp_tree_struct KPTree;
typedef struct kptree_grid_struct KPTreeGrid;

struct kp_tree_node_struct
{
    uint dim;
    uint idx;
    Point loc;
};

struct kp_tree_struct
{
    uint nnodes;
    KPTreeNode* nodes;
};

struct kptree_grid_struct
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

#ifdef INCLUDE_SOURCE
#include "kptree.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

