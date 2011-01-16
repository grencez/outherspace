
#ifndef KDTREE_H_
#define KDTREE_H_

#include "space.c"

struct kd_tree_leaf_struct;
struct kd_tree_inner_struct;
struct kd_tree_node_struct;
typedef struct kd_tree_leaf_struct  KDTreeLeaf;
typedef struct kd_tree_inner_struct KDTreeInner;
typedef struct kd_tree_node_struct  KDTreeNode;

struct kd_tree_leaf_struct
{
    uint nelems;
    BoundingBox box;
    const Triangle** elems;
};
struct kd_tree_inner_struct
{
    real split_pos;
    KDTreeNode* parent;
    KDTreeNode* children[2];
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
    BoundingBox box;
    KDTreeNode root;
};
typedef struct kd_tree_struct KDTree;


bool leaf_KDTreeNode (const KDTreeNode* node);

void output_KDTree (FILE* out, const KDTree* tree,
                    uint nelems, const Triangle* elems);

void cleanup_KDTree (KDTree* tree);

void build_KDTree (KDTree* tree, uint nelems, const Triangle** elems);

const KDTreeNode* upnext_KDTreeNode (Point* entrance,
                                     const KDTreeNode** parent_ptr,
                                     const Point* origin,
                                     const Point* dir,
                                     const KDTreeNode* node);

#endif

