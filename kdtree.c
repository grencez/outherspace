
#ifndef __OPENCL_VERSION__
#include "kdtree.h"
#include "order.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

#define MaxKDTreeDepth 50
    /* #define MaxKDTreeDepth 0 */

static
    void
output_KDTreeNode (FILE* out, uint node_idx,
                   uint depth, const KDTree* tree)
{
    const KDTreeNode* node;
    node = &tree->nodes[node_idx];
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
            index = tree->elemidcs[leaf->elemidcs + ei];
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
                               1+depth, tree);
    }
}

void output_KDTree (FILE* out, const KDTree* tree,
                    uint nelems, const Simplex* elems)
{
    uint i;
    fputs ("-- KDTree --\n", out);
    fputs ("- Elements -", out);
    UFor( i, nelems )
    {
        fprintf (out, "\n %u  ", i);
        output_Simplex (out, &elems[i]);
    }
    fputs ("\n- Nodes -\n", out);
    output_KDTreeNode (out, 0, 0, tree);

}

static
    void
output_gv_KDTreeNode (FILE* out, uint node_idx, const KDTree* tree)
{
    const KDTreeNode* node;
    node = &tree->nodes[node_idx];
    if (!leaf_KDTreeNode (node))
    {
        uint i;
        const KDTreeInner* inner;
        inner = &node->as.inner;
        UFor( i, 2 )
        {
            fprintf (out, "\"%u\" -> \"%u\";\n",
                     node_idx, inner->children[i]);
            output_gv_KDTreeNode (out, inner->children[i], tree);
        }
    }
}

void output_gv_KDTree (FILE* out, const KDTree* tree)
{
    fputs ("digraph kdtree {\n", out);
    fputs ("node [color=lightblue2, style=filled];\n", out);
    output_gv_KDTreeNode (out, 0, tree);
    fputs ("}\n", out);

}



static void cleanup_KDTreeNode (uint node_idx, KDTree* tree)
{
    KDTreeNode* node;
    node = &tree->nodes[node_idx];
    if (!leaf_KDTreeNode (node))
    {
        KDTreeInner* inner;
        inner = &node->as.inner;
        cleanup_KDTreeNode (inner->children[0], tree);
        cleanup_KDTreeNode (inner->children[1], tree);
    }
}

void init_KDTree (KDTree* tree)
{
    tree->nnodes = 0;
    tree->nelemidcs = 0;
}

void cleanup_KDTree (KDTree* tree)
{
    if (tree->nnodes > 0)
    {
        cleanup_KDTreeNode (0, tree);
        free (tree->nodes);
    }
    if (tree->nelemidcs > 0)
        free (tree->elemidcs);
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
    uint nbelow = 0, nabove = 0, split_dim = NDimensions;
    real split_pos = 0;
    bool split_low = false;

    nintls = grid->nintls;
    assert (even_uint (nintls));

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
                  uint depth,
                  SList* nodelist, SList* elemidxlist)
{
    KDTreeGrid logrid, higrid;
    KDTreeNode* node;
    node = AllocT( KDTreeNode, 1 );
    app_SList (nodelist, node);
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
        leaf->elemidcs = elemidxlist->nmembs;

        if (0 != leaf->nelems)
        {
            uint i;
            UFor( i, grid->nintls )
            {
                uint ti;
                ti = grid->intls[0][i];
                if (even_uint (ti))
                {
                    uint* idx_ptr;
                    idx_ptr = AllocT( uint, 1 );
                    *idx_ptr = ti / 2;
                    app_SList (elemidxlist, idx_ptr);
                }
            }
        }
        assert (leaf->elemidcs + leaf->nelems == elemidxlist->nmembs);
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
                output_Simplex (out, elems[i]);
            }
            fputc ('\n', out);
        }
#endif


        inner->children[0] = nodelist->nmembs;
        build_KDTreeNode (inner->children[0], node_idx,
                          &logrid, 1+depth, nodelist, elemidxlist);
        inner->children[1] = nodelist->nmembs;
        build_KDTreeNode (inner->children[1], node_idx,
                          &higrid, 1+depth, nodelist, elemidxlist);

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
            assert (leaf->nelems + leaf->elemidcs <= tree->nelemidcs);
            UFor( j, leaf->nelems )
            {
                uint idx;
                idx = tree->elemidcs[leaf->elemidcs + j];
                assert (idx < nelems);
                contains[idx] = true;
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

static int uintcmp (const void* a, const void* b)
{
    if (*(uint*)a < *(uint*)b)  return -1;
    if (*(uint*)a > *(uint*)b)  return  1;
    return 0;
}

    void
build_KDTree (KDTree* tree, KDTreeGrid* grid)
{
    uint i;
    SList nodelist, elemidxlist;

    UFor( i, NDimensions )
        sort_indexed_reals (grid->nintls, grid->intls[i], grid->coords[i]);
    init_SList (&nodelist);
    init_SList (&elemidxlist);
    build_KDTreeNode (0, 0, grid, 0, &nodelist, &elemidxlist);

    tree->nnodes = nodelist.nmembs;
    tree->nelemidcs = elemidxlist.nmembs;

    tree->nodes = AllocT( KDTreeNode, tree->nnodes );
    tree->elemidcs = AllocT( uint, tree->nelemidcs );

    unroll_SList (tree->nodes, &nodelist, sizeof (KDTreeNode));
    unroll_SList (tree->elemidcs, &elemidxlist, sizeof (uint));

    UFor( i, tree->nnodes )
    {
        if (leaf_KDTreeNode (&tree->nodes[i]))
        {
            KDTreeLeaf* leaf;
            leaf = &tree->nodes[i].as.leaf;
            qsort (&tree->elemidcs[leaf->elemidcs], leaf->nelems,
                   sizeof(uint), uintcmp);
        }
    }

    assert (complete_KDTree (tree, grid->nintls / 2));
}

#endif  /* #ifndef __OPENCL_VERSION__ */


bool leaf_KDTreeNode (__global const KDTreeNode* node)
{
    return node->split_dim == NDimensions;
}


uint find_KDTreeNode (uint* ret_parent,
                      const Point* origin,
                      __global const KDTreeNode* nodes)
{
    __global const KDTreeNode* node;
    uint node_idx = 0, parent = 0;
    assert (nodes);
    node = &nodes[0];
    while (! leaf_KDTreeNode (node))
    {
        __global const KDTreeInner* inner;
        inner = &node->as.inner;
        parent = node_idx;
        if (origin->coords[node->split_dim] < inner->split_pos)
            node_idx = inner->children[0];
        else
            node_idx = inner->children[1];
        node = &nodes[node_idx];
    }
    *ret_parent = parent;
    assert (leaf_KDTreeNode (node));
    return node_idx;
}


    uint
upnext_KDTreeNode (Point* entrance,
                   uint* ret_parent,
                   const Point* origin,
                   const Point* dir,
                   uint node_idx,
                   __global const KDTreeNode* nodes)
{
    __global const BoundingBox* box;
    uint child_idx;

    {
        __global const KDTreeNode* node;
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
        __global const KDTreeNode* node;
        __global const KDTreeInner* inner;
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


    uint
descend_KDTreeNode (uint* ret_parent,
                    const Point* entrance,
                    uint node_idx,
                    __global const KDTreeNode* nodes)
{
    __global const KDTreeNode* restrict node;
    node = &nodes[node_idx];

    while (! leaf_KDTreeNode (node))
    {
        __global const KDTreeInner* restrict inner;
        inner = &node->as.inner;
        *ret_parent = node_idx;

            /* Subtlety: Inclusive case here must be opposite of
             * inclusive case in upnext_KDTreeNode to avoid infinite
             * iteration on rays in the splitting plane's subspace.
             */
        if (entrance->coords[node->split_dim] <= inner->split_pos)
            node_idx = inner->children[0];
        else
            node_idx = inner->children[1];

        node = &nodes[node_idx];
    }

    return node_idx;
}

