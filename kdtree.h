
#ifndef KDTREE_H_
#ifndef __OPENCL_VERSION__
#define KDTREE_H_

#include "simplex.h"
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct KDTreeLeaf  KDTreeLeaf;
typedef struct KDTreeInner KDTreeInner;
typedef struct KDTreeNode  KDTreeNode;
typedef struct KDTree KDTree;
typedef struct KDTreeGrid KDTreeGrid;

struct KDTreeLeaf
{
    uint nelems;
    BoundingBox box;
    uint elemidcs;
};
struct KDTreeInner
{
    real split_pos;
    uint parent;
    uint children[2];
};
struct KDTreeNode
{
    uint split_dim;
    union KDTreeNode_union
    {
        KDTreeLeaf leaf;
        KDTreeInner inner;
    } as;
};

struct KDTree
{
    uint nnodes;
    uint nelemidcs;
    KDTreeNode* nodes;
    uint* elemidcs;
};

struct KDTreeGrid
{
    uint nelems;
    uint* elemidcs;
        /* /intls[dim]/ and /coords[dim]/ are sized to 2*nelems.*/
    uint* intls[NDimensions];
    real* coords[NDimensions];
    BoundingBox box;
};


bool leaf_KDTreeNode (__global const KDTreeNode* node);

uint
find_KDTreeNode (uint* ret_parent,
                 const Point* origin,
                 __global const KDTreeNode* nodes);
uint
first_KDTreeNode (uint* ret_parent,
                  const Ray* restrict ray,
                  __global const KDTreeNode* restrict nodes,
                  const BoundingBox* restrict box,
                  bool inside_box);
uint
next_KDTreeNode (uint* ret_parent,
                 const Ray* ray,
                 const Point* invdir,
                 real hit_mag,
                 uint node_idx,
                 __global const KDTreeNode* nodes);

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
cleanup_KDTree (KDTree* tree);
void
init_KDTreeGrid (KDTreeGrid* grid, uint nelems);
void
cleanup_KDTreeGrid (KDTreeGrid* grid);
void
shrink_KDTreeGrid (KDTreeGrid* grid, uint nelems);
void
build_trivial_KDTree (KDTree* tree, uint nelems, const BoundingBox* box);
void
build_KDTree (KDTree* tree, KDTreeGrid* grid, const Simplex* elems);

#ifdef INCLUDE_SOURCE
#include "kdtree.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

