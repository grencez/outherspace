
#ifndef KDTREE_H_
#ifndef __OPENCL_VERSION__
#define KDTREE_H_

#include "simplex.h"
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct kd_tree_leaf_struct;
struct kd_tree_inner_struct;
struct kd_tree_node_struct;
struct kd_tree_struct;
struct kdtree_grid_struct;
struct kp_tree_node_struct;
struct kp_tree_struct;
typedef struct kd_tree_leaf_struct  KDTreeLeaf;
typedef struct kd_tree_inner_struct KDTreeInner;
typedef struct kd_tree_node_struct  KDTreeNode;
typedef struct kd_tree_struct KDTree;
typedef struct kdtree_grid_struct KDTreeGrid;
typedef struct kp_tree_node_struct KPTreeNode;
typedef struct kp_tree_struct KPTree;
typedef struct kptree_grid_struct KPTreeGrid;

struct kd_tree_leaf_struct
{
    uint nelems;
    BoundingBox box;
    uint elemidcs;
};
struct kd_tree_inner_struct
{
    real split_pos;
    uint parent;
    uint children[2];
};
struct kd_tree_node_struct
{
    uint split_dim;
    union kd_tree_node_struct_union
    {
        KDTreeLeaf leaf;
        KDTreeInner inner;
    } as;
};

struct kd_tree_struct
{
    uint nnodes;
    uint nelemidcs;
    KDTreeNode* nodes;
    uint* elemidcs;
};

struct kdtree_grid_struct
{
    uint nintls;
    uint* intls[NDimensions];
    real* coords[NDimensions];
    BoundingBox box;
};

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


bool leaf_KDTreeNode (__global const KDTreeNode* node);

uint find_KDTreeNode (uint* ret_parent,
                      const Point* origin,
                      __global const KDTreeNode* nodes);

uint
upnext_KDTreeNode (Point* entrance,
                   uint* ret_parent,
                   const Point* origin,
                   const Point* dir,
                   uint node,
                   __global const KDTreeNode* nodes);
uint
descend_KDTreeNode (uint* ret_parent,
                    const Point* entrance,
                    uint node_idx,
                    __global const KDTreeNode* nodes);

uint
descend_KPTree (const KPTree* tree, const Point* loc, uint i);
uint
nearest_neighbor_KPTree (const KPTree* tree, const Point* loc);
#ifndef __OPENCL_VERSION__
void
output_KDTreeGrid (FILE* out, const KDTreeGrid* grid);
void
output_KDTree (FILE* out, const KDTree* tree);
void
output_simplex_KDTree (FILE* out, const KDTree* tree,
                       uint nelems, const Simplex* elems);
void output_gv_KDTree (FILE* out, const KDTree* tree);

void
init_KDTree (KDTree* tree);
void
init_KPTree (KPTree* tree);
void
cleanup_KDTree (KDTree* tree);
void
cleanup_KDTreeGrid (KDTreeGrid* grid);
void
cleanup_KPTree (KPTree* tree);
void
build_KDTree (KDTree* tree, KDTreeGrid* grid);
void
build_KPTree (KPTree* tree, KPTreeGrid* grid);

#ifdef INCLUDE_SOURCE
#include "kdtree.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

