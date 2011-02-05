
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

struct kdtree_grid_struct
{
    uint nintls;
    uint* intls[NDimensions];
    real* coords[NDimensions];
};
typedef struct kdtree_grid_struct KDTreeGrid;

static
    void
init_KDTreeGrid (KDTreeGrid* grid, uint nelems, const Triangle* elems)
{
    uint i;

    grid->nintls = 2 * nelems;
    grid->intls[0] = AllocT( uint, NDimensions * grid->nintls );
    grid->coords[0] = AllocT( real, NDimensions * grid->nintls );

    UFor( i, NDimensions )
    {
        grid->intls[i] = &grid->intls[0][i * grid->nintls];
        grid->coords[i] = &grid->coords[0][i * grid->nintls];
    }

    UFor( i, nelems )
    {
        uint ti, dim;
        const Triangle* elem;

        ti = 2*i;
        elem = &elems[i];

        UFor( dim, NDimensions )
        {
            uint pi;
            real min_coord = Max_real;
            real max_coord = Min_real;

            UFor( pi, NTrianglePoints )
            {
                if (elem->pts[pi].coords[dim] < min_coord)
                    min_coord = elem->pts[pi].coords[dim];
                if (elem->pts[pi].coords[dim] > max_coord)
                    max_coord = elem->pts[pi].coords[dim];
            }

            grid->intls[dim][ti] = ti;
            grid->intls[dim][ti+1] = ti+1;
            grid->coords[dim][ti] = min_coord;
            grid->coords[dim][ti+1] = max_coord;
        }
    }
}

static
    void
cleanup_KDTreeGrid (KDTreeGrid* grid)
{
    free (grid->intls[0]);
    free (grid->coords[0]);
}

static
    void
split_KDTreeGrid (KDTreeGrid* logrid, KDTreeGrid* higrid,
                  KDTreeGrid* grid,
                  uint split_dim, real split_pos)
{
    uint dim, nintls;
    const real* bounds;

    assert (split_dim < NDimensions);
    nintls = grid->nintls;
    bounds = grid->coords[split_dim];

    assert (even_uint (logrid->nintls));
    assert (even_uint (higrid->nintls));

    logrid->intls[0] = AllocT( uint, NDimensions * logrid->nintls );
    higrid->intls[0] = AllocT( uint, NDimensions * higrid->nintls );

    UFor( dim, NDimensions )
    {
        uint i, loidx = 0, hiidx = 0;
        uint* lointls;
        uint* hiintls;

        logrid->intls[dim] = &logrid->intls[0][dim * logrid->nintls];
        higrid->intls[dim] = &higrid->intls[0][dim * higrid->nintls];

        logrid->coords[dim] = grid->coords[dim];
        higrid->coords[dim] = grid->coords[dim];

        lointls = logrid->intls[dim];
        hiintls = higrid->intls[dim];

        UFor( i, nintls )
        {
            uint ti, loti, hiti;
            ti = lointls[i];
            if (even_uint (ti))
            {
                loti = ti;
                hiti = ti + 1;
            }
            else
            {
                loti = ti - 1;
                hiti = ti;
            }

            if (bounds[loti] < split_pos)  lointls[loidx++] = ti;
            if (bounds[hiti] > split_pos)  hiintls[hiidx++] = ti;
        }

        if (dim == 0)
        {
            assert (loidx <= logrid->nintls);
            assert (hiidx <= higrid->nintls);
            assert (even_uint (loidx));
            assert (even_uint (hiidx));
            if (loidx < logrid->nintls)  logrid->nintls = loidx;
            if (hiidx < higrid->nintls)  higrid->nintls = hiidx;
        }
        else
        {
            assert (loidx == logrid->nintls);
            assert (hiidx == higrid->nintls);
        }
    }
}

static
    void
sort_indexed_reals (uint nmembs, uint* indices, real* membs)
{
    uint i;
    UFor( i, nmembs )
    {
        uint j, ti;
        ti = indices[i];
        UFor( j, i )
        {
            uint tj;
            tj = indices[j];

            if (membs[tj] > membs[ti])
            {
                indices[i] = tj;
                indices[j] = ti;
                ti = tj;
            }
        }
    }
    if (nmembs > 1)
    {
        UFor( i, nmembs-1 )
            assert (membs[indices[i]] <= membs[indices[i+1]]);
    }
}

static
    real
kdtree_cost_fn (uint split_dim, real split_pos,
                uint nlo, uint nhi,
                const BoundingBox* box)
{
    const real cost_it = 1;  /* Cost of intersection test.*/
    const real cost_tr = 8;  /* Cost of traversal.*/
    BoundingBox lo_box, hi_box;

    split_BoundingBox (&lo_box, &hi_box, box, split_dim, split_pos);

    return (cost_tr
            + cost_it
            * (nlo * surface_area_BoundingBox (&lo_box) +
               nhi * surface_area_BoundingBox (&hi_box))
            / surface_area_BoundingBox (box));
}

static
    bool
determine_split (KDTreeGrid* logrid, KDTreeGrid* higrid,
                 KDTreeGrid* grid, const BoundingBox* box)
{
    const real cost_it = 1;  /* Cost of intersection test.*/
    uint nelems, nintls;
    uint dim;
    real cost_split, cost_nosplit;
    uint nbelow, nabove, split_dim;
    real split_pos;

    assert (even_uint (grid->nintls));
    nintls = grid->nintls;
    nelems = nintls / 2;
    cost_split = Max_real;
    cost_nosplit = cost_it * nelems;

    UFor( dim, NDimensions )
    {
        uint i, nlo, nhi;  /* Element counts below and above split line.*/
        real lo_box, hi_box;
        const uint* intls;
        const real* coords;

        nlo = 0;
        nhi = nelems;
        lo_box = box->min_corner.coords[dim];
        hi_box = box->max_corner.coords[dim];
        intls = grid->intls[dim];
        coords = grid->coords[dim];

        UFor( i, nintls )
        {
            uint ti;
            ti = intls[i];
            if (coords[i] <= lo_box)
            {
                assert (even_uint (ti));
                ++ nlo;
            }
            else if (coords[i] >= hi_box)
            {
                assert (!even_uint (ti));
                -- nhi;
            }
            else
            {
                real cost;
                if (even_uint (ti))
                {
                    cost = kdtree_cost_fn (dim, coords[ti], nlo, nhi, box);
                    ++ nlo;
                }
                else
                {
                    -- nhi;
                    cost = kdtree_cost_fn (dim, coords[ti], nlo, nhi, box);
                }

                if (cost < cost_split)
                {
                    cost_split = cost;
                    split_dim = dim;
                    split_pos = coords[ti];
                    nbelow = nlo;
                    nabove = nhi;
                }
            }
        }
    }

    if (cost_split > cost_nosplit)  return false;

        /* At this point, split_dim and split_pos are known.*/
    logrid->nintls = 2 * nbelow;
    higrid->nintls = 2 * nabove;
    split_KDTreeGrid (logrid, higrid, grid, split_dim, split_pos);

    return true;
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
    KDTreeGrid grid;

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

    init_KDTreeGrid (&grid, nelems, selems);

    UFor( i, NDimensions )
        sort_indexed_reals (nelems, grid.intls[i], grid.coords[i]);

    init_SList (&lis);
    build_KDTreeNode (0, 0, &tree->box, nelems, elems, 0, selems, &lis);

    tree->nnodes = lis.nmembs;
    tree->nodes = AllocT( KDTreeNode, tree->nnodes );
    unroll_SList (tree->nodes, &lis, sizeof (KDTreeNode));

    cleanup_KDTreeGrid (&grid);
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

