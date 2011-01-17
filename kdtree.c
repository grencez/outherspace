
#include "kdtree.h"

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
output_KDTreeNode (FILE* out, const KDTreeNode* node,
                   uint depth, uint id, const Triangle* elems)
{
    fprintf (out, " %*s%-*u depth:%u   ", depth, "", depth+1, id, depth);
    if (leaf_KDTreeNode (node))
    {
        uint ei;
        const KDTreeLeaf* leaf = &node->as.leaf;
        output_BoundingBox (out, &leaf->box);
        fprintf (out, "\n %*selems:", 2*depth+2, "");
        UFor( ei, leaf->nelems )
        {
            uint index;
            index = index_of (leaf->elems[ei], elems, sizeof (Triangle));
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
                               1+depth,
                               2*(id+1)-1 + i,
                               elems);
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
    output_KDTreeNode (out, &tree->root, 0, 0, elems);
}


static void cleanup_KDTreeNode (KDTreeNode* node)
{
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
        cleanup_KDTreeNode (inner->children[0]);
        cleanup_KDTreeNode (inner->children[1]);
        free (inner->children[0]);
    }
}

void cleanup_KDTree (KDTree* tree)
{
    cleanup_KDTreeNode (&tree->root);
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
build_KDTreeNode (KDTreeNode* node,
                  KDTreeNode* parent, BoundingBox* box,
                  uint nelems, const Triangle** elems,
                  uint depth)
{
        /* printf ("%*sdepth=%u, nelems=%u\n", depth, "", depth, nelems); */
    if (depth < MaxKDTreeDepth)
    {
        uint nbelow, nabove;
        KDTreeNode** children;
        KDTreeInner* inner;
        inner = &node->as.inner;
        children = inner->children;
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

        children[0] = (KDTreeNode*) malloc (2 * sizeof (KDTreeNode));
        children[1] = &children[0][1];

        inner->parent = parent;

        {
            real tmp;
            const Triangle** blw_elems;

            blw_elems = (const Triangle**)
                malloc (nbelow * sizeof (Triangle*));
            memcpy (blw_elems, elems, nbelow * sizeof (Triangle*));

            tmp = box->max_corner.coords[node->split_dim];
            box->max_corner.coords[node->split_dim] = inner->split_pos;
            build_KDTreeNode (children[0], node, box, nbelow,
                              blw_elems, 1+depth);
            box->max_corner.coords[node->split_dim] = tmp;

            free (blw_elems);

            tmp = box->min_corner.coords[node->split_dim];
            box->min_corner.coords[node->split_dim] = inner->split_pos;
            build_KDTreeNode (children[1], node, box, nabove,
                              &elems[nelems-nabove], 1+depth);
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
            leaf->elems = (const Triangle**) malloc (nelems * sizeof (Triangle*));
            memcpy (leaf->elems, elems, nelems * sizeof (Triangle*));
        }
    }
}

void build_KDTree (KDTree* tree, uint nelems, const Triangle** elems)
{
    uint i, ei;
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

    build_KDTreeNode (&tree->root, 0, &tree->box, nelems, elems, 0);
}

const KDTreeNode* find_KDTreeNode (const KDTreeNode** parent_ptr,
                                   const Point* origin,
                                   const KDTree* tree)
{
    const KDTreeNode* node;
    const KDTreeNode* parent = 0;
    node = &tree->root;
    while (! leaf_KDTreeNode (node))
    {
        const KDTreeInner* inner;
        inner = &node->as.inner;
        parent = node;
        if (origin->coords[node->split_dim] < inner->split_pos)
            node = inner->children[0];
        else
            node = inner->children[1];
    }
    *parent_ptr = parent;
    return node;
}

const KDTreeNode* upnext_KDTreeNode (Point* entrance,
                                     const KDTreeNode** parent_ptr,
                                     const Point* origin,
                                     const Point* dir,
                                     const KDTreeNode* node)
{
    const BoundingBox* box;
    const KDTreeNode* child;

    assert (leaf_KDTreeNode (node));
    box = &node->as.leaf.box;
    child = node;
    assert (parent_ptr);
    node = *parent_ptr;

    while (1)
    {
        const KDTreeInner* inner;
        tristate facing;
        uint ni;

            /* Backtracked from root node => no possible next leaf.*/
        if (!node)
        {
            child = 0;
            break;
        }

        assert (! leaf_KDTreeNode (node));
        inner = &node->as.inner;

        facing = signum_real (dir->coords[node->split_dim]);

            /* Subtlety: Inclusive case opposite when descending tree.*/
        ni = (facing < 0) ? 0 : 1;

            /* Ray pointing to previously visited node => keep backtracking.*/
        if (inner->children[ni] != child)
        {
            if (hit_BoundingPlane (entrance,
                                   node->split_dim, inner->split_pos,
                                   box, origin, dir))
            {
                *parent_ptr = node;
                child = inner->children[ni];
                break;
            }
        }

        child = node;
        node = inner->parent;
    }
    return child;
}

