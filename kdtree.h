
#ifndef KDTREE_H_
#ifndef __OPENCL_VERSION__
#define KDTREE_H_

#include "scene.h"
#include "simplex.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct kd_tree_leaf_struct;
struct kd_tree_inner_struct;
struct kd_tree_node_struct;
struct kd_tree_struct;
struct kdtree_grid_struct;
typedef struct kd_tree_leaf_struct  KDTreeLeaf;
typedef struct kd_tree_inner_struct KDTreeInner;
typedef struct kd_tree_node_struct  KDTreeNode;
typedef struct kd_tree_struct KDTree;
typedef struct kdtree_grid_struct KDTreeGrid;

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


bool leaf_KDTreeNode (__global const KDTreeNode* node);

uint find_KDTreeNode (uint* ret_parent,
                      const Point* origin,
                      __global const KDTreeNode* nodes);

uint upnext_KDTreeNode (Point* entrance,
                        uint* ret_parent,
                        const Point* origin,
                        const Point* dir,
                        uint node,
                        __global const KDTreeNode* nodes);

#ifndef __OPENCL_VERSION__
void output_KDTree (FILE* out, const KDTree* tree,
                    uint nelems, const Triangle* elems);
void output_gv_KDTree (FILE* out, const KDTree* tree);

void cleanup_KDTree (KDTree* tree);

void
init_Triangle_KDTreeGrid (KDTreeGrid* grid,
                          uint nelems, const Triangle* elems,
                          const BoundingBox* box);
void
build_KDTree (KDTree* tree, KDTreeGrid* grid);

#ifdef INCLUDE_SOURCE
#include "kdtree.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

