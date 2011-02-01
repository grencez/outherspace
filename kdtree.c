
#include "kdtree.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

#define MaxKDTreeDepth 2*NDimensions
    /* #define MaxKDTreeDepth 0 */


bool leaf_KDTreeNode (const KDTreeNode* node)
{
    return node->split_dim == NDimensions;
}

static
    void
output_KDTreeNode (FILE* out, uint node_idx,
                   uint depth, const KDTreeNode* nodes)
{
    const KDTreeNode* node;
    node = &nodes[node_idx];
    fprintf (out, " %*s%-*u depth:%u   ",
             depth, "", depth+1,
             node_idx, depth);
    if (leaf_KDTreeNode (node))
    {
        uint ei;
        const KDTreeLeaf* leaf = &node->as.leaf;
        output_BoundingBox (out, &leaf->box);
        fprintf (out, "\n %*selems:", 2*depth+2, "");
        UFor( ei, leaf->nelems )
        {
            uint index;
            index = leaf->elems[ei];
            fprintf (out, " %u", index);
        }
        fputc ('\n', out);

    }
    else
    {
        uint i;
        const KDTreeInner* inner;
        inner = &node->as.inner;
        fprintf (out, "dim:%u  pos:%.1f\n",
                 node->split_dim, inner->split_pos);
        UFor( i, 2 )
            output_KDTreeNode (out, inner->children[i],
                               1+depth, nodes);
    }
}

void output_KDTree (FILE* out, const KDTree* tree,
                    uint nelems, const Triangle* elems)
{
    uint i;
    fputs ("-- KDTree --\n", out);
    fputs ("- Elements -", out);
    UFor( i, nelems )
    {
        fprintf (out, "\n %u  ", i);
        output_Triangle (out, &elems[i]);
    }
    fputs ("\n- Nodes -\n", out);
    output_KDTreeNode (out, 0, 0, tree->nodes);

}


static void cleanup_KDTreeNode (uint node_idx, KDTreeNode* nodes)
{
    KDTreeNode* node;
    node = &nodes[node_idx];
    if (leaf_KDTreeNode (node))
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        if (0 != leaf->nelems)
            free (leaf->elems);
    }
    else
    {
        KDTreeInner* inner;
        inner = &node->as.inner;
        cleanup_KDTreeNode (inner->children[0], nodes);
        cleanup_KDTreeNode (inner->children[1], nodes);
    }
}

void cleanup_KDTree (KDTree* tree)
{
    if (tree->nodes)
    {
        cleanup_KDTreeNode (0, tree->nodes);

        free (tree->nodes);
    }
}

    /* -1 strictly below, 0 in both, 1 strictly greater or equal */
static
    tristate
splitting_plane (const Triangle* elem, uint split_dim, real split_pos)
{
    bool below = false;
    bool above = false;
    uint pi;
    UFor( pi, NTrianglePoints )
        if (elem->pts[pi].coords[split_dim] < split_pos)
            below = true;
        else
            above = true;
    if (below && above)  return 0;
    if (below)           return -1;
    if (above)           return 1;
    assert (0);
    return 0;
}

static 
    void
partition_split (uint nelems, const Triangle** elems,
                 uint* nbelow, uint* nabove, const KDTreeNode* node)
{
    uint ei, l, r;
    ei = 0;
    l = 0;
    r = nelems;

    while (ei < r)
    {
        tristate pos;
        pos = splitting_plane (elems[ei],
                               node->split_dim, node->as.inner.split_pos);
        if (pos < 0)
        {
            const Triangle* tmp;
            tmp = elems[l];
            elems[l] = elems[ei];
            elems[ei] = tmp;
            ++ l;
            ++ ei;
        }
        if (pos > 0)
        {
            const Triangle* tmp;
            -- r;
            tmp = elems[r];
            elems[r] = elems[ei];
            elems[ei] = tmp;
        }
        if (pos == 0)
        {
            ++ ei;
        }
    }
    *nbelow = r;
    *nabove = nelems - l;
}

static
    void
build_KDTreeNode (uint node_idx,
                  uint parent, BoundingBox* box,
                  uint nelems, const Triangle** elems,
                  uint depth, const Triangle* selems,
                  SList* lis)
{
    KDTreeNode* node;
    node = AllocT( KDTreeNode, 1 );
    app_SList (lis, node);
        /* printf ("%*sdepth=%u, nelems=%u\n", depth, "", depth, nelems); */
    if (depth < MaxKDTreeDepth)
    {
        uint nbelow, nabove;
        KDTreeInner* inner;
        inner = &node->as.inner;
        node->split_dim = depth % NDimensions;
            /* node->split_dim = depth % 2; */
        inner->split_pos = 0.5 * (box->min_corner.coords[node->split_dim] +
                                  box->max_corner.coords[node->split_dim]);
        partition_split (nelems, elems, &nbelow, &nabove, node);

#if 0
        {
            uint i;
            FILE* out = stderr;
            fprintf (out, "%*sdim:%u pos:%.1f  blw:%u abv:%u\n",
                     2*depth, "",
                     node->split_dim, inner->split_pos,
                     nbelow, nabove);
            fprintf (out, "%*s", 2*depth+1, "");
            output_BoundingBox (out, box);
            UFor( i, nelems )
            {
                fprintf (out, "\n%*s", 2*depth+1, "");
                output_Triangle (out, elems[i]);
            }
            fputc ('\n', out);
        }
#endif

        inner->parent = parent;

        {
            real tmp;
            const Triangle** blw_elems;

            blw_elems = (const Triangle**)
                malloc (nbelow * sizeof (Triangle*));
            memcpy (blw_elems, elems, nbelow * sizeof (Triangle*));

            tmp = box->max_corner.coords[node->split_dim];
            box->max_corner.coords[node->split_dim] = inner->split_pos;
            inner->children[0] = lis->nmembs;
            build_KDTreeNode (inner->children[0], node_idx,
                              box, nbelow, blw_elems,
                              1+depth, selems, lis);
            box->max_corner.coords[node->split_dim] = tmp;

            free (blw_elems);

            tmp = box->min_corner.coords[node->split_dim];
            box->min_corner.coords[node->split_dim] = inner->split_pos;
            inner->children[1] = lis->nmembs;
            build_KDTreeNode (inner->children[1], node_idx,
                              box, nabove, &elems[nelems-nabove],
                              1+depth, selems, lis);
            box->min_corner.coords[node->split_dim] = tmp;
        }

    }
    else
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        node->split_dim = NDimensions;
        memcpy (&leaf->box, box, sizeof (BoundingBox));
        leaf->nelems = nelems;
        if (0 != nelems)
        {
            uint i;
            leaf->elems = AllocT( uint, nelems );
            UFor( i, nelems )
                leaf->elems[i] = index_of (elems[i], selems, sizeof (Triangle));
        }
    }
}

void build_KDTree (KDTree* tree, uint nelems, const Triangle** elems,
                   const Triangle* selems)
{
    uint i, ei;
    SList lis;

    assert (nelems > 0);
    UFor( i, NDimensions )
    {
        real x;
        x = elems[0]->pts[0].coords[i];
        tree->box.min_corner.coords[i] = x;
        tree->box.max_corner.coords[i] = x;
        tree->box.min_corner.coords[i] = 0;
        tree->box.max_corner.coords[i] = 100;
    }

    UFor( ei, nelems )
    {
        uint pi;
        UFor( pi, NTrianglePoints )
            adjust_BoundingBox (&tree->box, &elems[ei]->pts[pi]);
    }

    UFor( ei, nelems )
    {
        uint pi;
        UFor( pi, NTrianglePoints )
            assert (inside_BoundingBox (&tree->box, &elems[ei]->pts[pi]));
    }

    init_SList (&lis);
    build_KDTreeNode (0, 0, &tree->box, nelems, elems, 0, selems, &lis);

    tree->nnodes = lis.nmembs;
    tree->nodes = AllocT( KDTreeNode, tree->nnodes );
    acpy_SList (tree->nodes, &lis, sizeof (KDTreeNode));
    cleanup_SList (&lis);
}

uint find_KDTreeNode (uint* ret_parent,
                      const Point* origin,
                      const KDTree* tree)
{
    const KDTreeNode* node;
    uint node_idx = 0, parent = 0;
    assert (tree->nnodes > 0);
    node = &tree->nodes[0];
    while (! leaf_KDTreeNode (node))
    {
        const KDTreeInner* inner;
        inner = &node->as.inner;
        parent = node_idx;
        if (origin->coords[node->split_dim] < inner->split_pos)
            node_idx = inner->children[0];
        else
            node_idx = inner->children[1];
        node = &tree->nodes[node_idx];
    }
    *ret_parent = parent;
    assert (leaf_KDTreeNode (node));
    return node_idx;
}

uint upnext_KDTreeNode (Point* entrance,
                        uint* ret_parent,
                        const Point* origin,
                        const Point* dir,
                        uint node_idx,
                        const KDTreeNode* nodes)
{
    const BoundingBox* box;
    uint child_idx;

    {
        const KDTreeNode* node;
        node = &nodes[node_idx];
        assert (leaf_KDTreeNode (node));
        box = &node->as.leaf.box;
    }

    child_idx = node_idx;
    assert (ret_parent);
    node_idx = *ret_parent;

        /* Terminating condition:
         * Backtracked from root node => no possible next leaf.
         */
    while (node_idx != child_idx)
    {
        const KDTreeNode* node;
        const KDTreeInner* inner;
        tristate facing;
        uint ni;

        node = &nodes[node_idx];

        assert (! leaf_KDTreeNode (node));
        inner = &node->as.inner;

        facing = signum_real (dir->coords[node->split_dim]);

            /* Subtlety: Inclusive case opposite when descending tree.*/
        ni = (facing < 0) ? 0 : 1;

            /* Ray pointing to previously visited node => keep backtracking.*/
        if (inner->children[ni] != child_idx)
        {
            if (hit_inner_BoundingPlane (entrance,
                                         node->split_dim, inner->split_pos,
                                         box, origin, dir))
            {
                child_idx = inner->children[ni];
                break;
            }
        }

        child_idx = node_idx;
        node_idx = inner->parent;
    }
    *ret_parent = node_idx;
    return child_idx;
}

