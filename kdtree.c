
#include "kdtree.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

#define MaxKDTreeDepth 50
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

struct kdtree_grid_struct
{
    uint nintls;
    uint* intls[NDimensions];
    real* coords[NDimensions];
    BoundingBox box;
};
typedef struct kdtree_grid_struct KDTreeGrid;

static
    void
init_KDTreeGrid (KDTreeGrid* grid, uint nelems, const Triangle* elems,
                 const BoundingBox* box)
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
    copy_BoundingBox (&grid->box, box);
}

static
    void
cleanup_KDTreeGrid (KDTreeGrid* grid)
{
    if (grid->nintls > 0)
    {
        free (grid->intls[0]);
        free (grid->coords[0]);
    }
}

static
    void
split_KDTreeGrid (KDTreeGrid* logrid, KDTreeGrid* higrid,
                  KDTreeGrid* grid,
                  uint split_dim, real split_pos, bool split_low)
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
        const uint* intls;
        uint* lointls;
        uint* hiintls;

        logrid->coords[dim] = grid->coords[dim];
        higrid->coords[dim] = grid->coords[dim];

        logrid->intls[dim] = &logrid->intls[0][dim * logrid->nintls];
        higrid->intls[dim] = &higrid->intls[0][dim * higrid->nintls];

        intls = grid->intls[dim];
        lointls = logrid->intls[dim];
        hiintls = higrid->intls[dim];

        UFor( i, nintls )
        {
            uint ti, loti, hiti;
            ti = intls[i];
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

            if (bounds[loti] == split_pos && bounds[hiti] == split_pos)
            {
                if (split_low)
                {
                    assert (loidx < logrid->nintls);
                    lointls[loidx++] = ti;
                }
                else
                {
                    assert (hiidx < higrid->nintls);
                    hiintls[hiidx++] = ti;
                }
            }
            else
            {
                if (bounds[loti] < split_pos)
                {
                    assert (loidx < logrid->nintls);
                    lointls[loidx++] = ti;
                }
                if (bounds[hiti] > split_pos)
                {
                    assert (hiidx < higrid->nintls);
                    hiintls[hiidx++] = ti;
                }
            }
        }

        assert (loidx == logrid->nintls);
        assert (hiidx == higrid->nintls);
    }

    split_BoundingBox (&logrid->box, &higrid->box, &grid->box,
                       split_dim, split_pos);
}

static
    bool
minimal_unique (uint n, const uint* a)
{
    uint i;
    bool* hits;
    bool pred = true;

    hits = AllocT( bool, n );

    UFor( i, n )  hits[i] = false;
    UFor( i, n )
    {
        if (a[i] < n)
            hits[a[i]] = true;
    }
    UFor( i, n )
    {
        if (!hits[i])
            pred = false;
    }

    if (hits)  free (hits);
    return pred;
}


static
    void
sort_intervals (uint nintls, uint* intls, const real* coords)
{
    uint i;
    assert (even_uint (nintls));
    assert (minimal_unique (nintls, intls));
    UFor( i, nintls )
    {
        uint j, ti;
        ti = intls[i];
        UFor( j, i )
        {
            uint tj;
            tj = intls[j];

            if (coords[tj] > coords[ti])
            {
                intls[i] = tj;
                intls[j] = ti;
                ti = tj;
                assert (intls[i] != intls[j]);
            }
        }
    }
    assert (minimal_unique (nintls, intls));
    if (nintls > 0)
    {
        UFor( i, nintls-1 )
        {
            uint ti, tj;
            ti = intls[i];
            tj = intls[i+1];
            assert (coords[ti] <= coords[tj]);
        }
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
    uint
determine_split (KDTreeGrid* logrid, KDTreeGrid* higrid, KDTreeGrid* grid)
{
    const real cost_it = 1;  /* Cost of intersection test.*/
    uint nelems, nintls;
    uint dim;
    real cost_split, cost_nosplit;
    uint nbelow, nabove, split_dim;
    real split_pos;
    bool split_low;

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
        uint nend = 0, nali = 0, nbeg = 0;

        nlo = 0;
        nhi = nelems;
        lo_box = grid->box.min_corner.coords[dim];
        hi_box = grid->box.max_corner.coords[dim];
        intls = grid->intls[dim];
        coords = grid->coords[dim];

        UFor( i, nintls )
        {
            uint ti;
            ti = intls[i];
            if (even_uint (ti))
            {
                if (coords[ti] == coords[ti+1])  nali += 1;
                else                             nbeg += 1;
            }
            else
            {
                if (coords[ti] == coords[ti-1])  nali += 1;
                else                             nend += 1;
            }

            if (i == nintls -1 || coords[ti] != coords[intls[i+1]])
            {
                bool eval_cost;
                eval_cost = (coords[ti] > lo_box && coords[ti] < hi_box);

                assert (nhi >= nend);
                nhi -= nend;

                if (eval_cost)
                {
                    real cost;
                    cost = kdtree_cost_fn (dim, coords[ti], nlo, nhi,
                                           &grid->box);

                    if (cost < cost_split)
                    {
                        cost_split = cost;
                        split_dim = dim;
                        split_pos = coords[ti];
                        split_low = false;
                        nbelow = nlo;
                        nabove = nhi;
                    }
                }


                if (nali > 0)
                {
                    assert (even_uint (nali));
                    nali /= 2;

                    assert (nhi >= nali);
                    assert (nlo + nali <= nelems);
                    nhi -= nali;
                    nlo += nali;

                    if (eval_cost)
                    {
                        real cost;
                        cost = kdtree_cost_fn (dim, coords[ti], nlo, nhi,
                                               &grid->box);
                        if (cost < cost_split)
                        {
                            cost_split = cost;
                            split_dim = dim;
                            split_pos = coords[ti];
                            split_low = true;
                            nbelow = nlo;
                            nabove = nhi;
                        }
                    }
                }

                assert (nlo + nbeg <= nelems);
                nlo += nbeg;
                nend = 0; nali = 0; nbeg = 0;
            }
        }
        assert (nlo == nelems);
        assert (nhi == 0);
    }

    if (cost_split > cost_nosplit)  return NDimensions;

        /* At this point, the split is known.*/
    logrid->nintls = 2 * nbelow;
    higrid->nintls = 2 * nabove;
    split_KDTreeGrid (logrid, higrid, grid, split_dim, split_pos, split_low);

    return split_dim;
}

static
    void
build_KDTreeNode (uint node_idx, uint parent,
                  KDTreeGrid* grid,
                  uint depth, SList* lis)
{
    KDTreeGrid logrid, higrid;
    KDTreeNode* node;
    node = AllocT( KDTreeNode, 1 );
    app_SList (lis, node);
        /* printf ("%*sdepth=%u, nelems=%u\n", depth, "", depth, nelems); */

    if (depth >= MaxKDTreeDepth)
    {
        node->split_dim = NDimensions;
    }
    else
    {
        node->split_dim = determine_split (&logrid, &higrid, grid);
    }

    if (node->split_dim == NDimensions)
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        copy_BoundingBox (&leaf->box, &grid->box);

        assert (even_uint (grid->nintls));
        leaf->nelems = grid->nintls / 2;

        if (0 != leaf->nelems)
        {
            uint i, li = 0;
            leaf->elems = AllocT( uint, leaf->nelems );
            UFor( i, grid->nintls )
            {
                uint ti;
                ti = grid->intls[0][i];
                if (even_uint (ti))  leaf->elems[li++] = ti / 2;
            }
        }
    }
    else
    {
        KDTreeInner* inner;
        inner = &node->as.inner;
        inner->parent = parent;
        inner->split_pos = logrid.box.max_corner.coords[node->split_dim];

#if 0
        {
            uint i;
            FILE* out = stderr;
            fprintf (out, "%*sdim:%u pos:%.1f  blw:%u abv:%u\n",
                     2*depth, "",
                     node->split_dim, inner->split_pos,
                     logrid->nintls/2, higrid->nintls/2);
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


        inner->children[0] = lis->nmembs;
        build_KDTreeNode (inner->children[0], node_idx,
                          &logrid, 1+depth, lis);
        inner->children[1] = lis->nmembs;
        build_KDTreeNode (inner->children[1], node_idx,
                          &higrid, 1+depth, lis);

        if (logrid.nintls > 0)  free (logrid.intls[0]);
        if (higrid.nintls > 0)  free (higrid.intls[0]);
    }
}

static
    bool
complete_KDTree (const KDTree* tree, uint nelems)
{
    uint i;
    bool* contains;
    bool pred = true;
    contains = AllocT( bool, nelems );

    UFor( i, nelems )  contains[i] = false;
    UFor( i, tree->nnodes )
    {
        if (leaf_KDTreeNode (&tree->nodes[i]))
        {
            uint j;
            const KDTreeLeaf* leaf;
            leaf = &tree->nodes[i].as.leaf;
            UFor( j, leaf->nelems )
            {
                assert (leaf->elems[j] < nelems);
                contains[leaf->elems[j]] = true;
            }
        }
    }

    UFor( i, nelems )
    {
        if (!contains[i])
            pred = false;
    }

    if (contains)  free (contains);
    return pred;
}

void build_KDTree (KDTree* tree, uint nelems, const Triangle* elems,
                   const BoundingBox* box)
{
    uint i, ei;
    SList lis;
    KDTreeGrid grid;

    assert (nelems > 0);

    UFor( ei, nelems )
    {
        uint pi;
        UFor( pi, NTrianglePoints )
            assert (inside_BoundingBox (box, &elems[ei].pts[pi]));
    }

    init_KDTreeGrid (&grid, nelems, elems, box);

    UFor( i, NDimensions )
        sort_intervals (grid.nintls, grid.intls[i], grid.coords[i]);

    init_SList (&lis);
    build_KDTreeNode (0, 0, &grid, 0, &lis);

    tree->nnodes = lis.nmembs;
    tree->nodes = AllocT( KDTreeNode, tree->nnodes );
    unroll_SList (tree->nodes, &lis, sizeof (KDTreeNode));

    cleanup_KDTreeGrid (&grid);
    assert (complete_KDTree (tree, nelems));
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

