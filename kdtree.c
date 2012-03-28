
#ifndef __OPENCL_VERSION__
#include "kdtree.h"

#include "bitstring.h"
#include "order.h"
#include "point.h"
#include "serial.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

#define MaxKDTreeDepth 50
    /* #define MaxKDTreeDepth 0 */

    void
output_KDTreeGrid (FILE* out, const KDTreeGrid* grid)
{
    uint dim;
    fputs ("-- KDTreeGrid --", out);
    fprintf (out, "nelems:%u\n", grid->nelems);
    UFor( dim, NDimensions )
    {
        uint idx;
        UFor( idx, grid->nelems )
        {
            uint ti;
            ti = grid->intls[dim][idx];
            fprintf (out, "dim:%u  idx:%u  ti:%u  elem:%u  coord:%f\n",
                     dim, idx, ti,
                     grid->elemidcs[ti/2],
                     grid->coords[dim][ti]);
        }
    }
    output_BBox (out, &grid->box);
    fputc ('\n', out);
}

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
        output_BBox (out, &leaf->box);
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

    void
output_KDTree (FILE* out, const KDTree* tree)
{
    fputs ("-- KDTree --\n", out);
    fputs ("\n- Nodes -\n", out);
    output_KDTreeNode (out, 0, 0, tree);
}

    void
output_simplex_KDTree (FILE* out, const KDTree* tree,
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

static
    void
cleanup_KDTreeNode (uint node_idx, KDTree* tree)
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

    void
init_KDTreeGrid (KDTreeGrid* grid, uint nelems)
{
    uint dim, nintls;
    nintls = 2 * nelems;

    grid->nelems = nelems;
    grid->elemidcs = AllocT( uint, nelems );
    grid->intls[0] = AllocT( uint, NDimensions * nintls );
    grid->coords[0] = AllocT( real, NDimensions * nintls );

    UFor( dim, NDimensions-1 )
    {
        grid->intls[dim+1] = &grid->intls[dim][nintls];
        grid->coords[dim+1] = &grid->coords[dim][nintls];
    }
}

    void
cleanup_KDTreeGrid (KDTreeGrid* grid)
{
    if (grid->nelems > 0)
    {
        free (grid->elemidcs);
        free (grid->intls[0]);
        free (grid->coords[0]);
    }
}

    void
shrink_KDTreeGrid (KDTreeGrid* grid, uint nelems)
{
    assert (nelems <= grid->nelems);
    grid->nelems = nelems;
}

static
    void
splitclip_Simplex_BBox (BBox* restrict lobox,
                        BBox* restrict hibox,
                        const BBox* restrict box,
                        const Simplex* restrict elem,
                        uint split_dim, real split_pos)
{
    bool in_lo = false, in_hi = false;
    uint pi;
    real diffs[NDimensions];

    UFor( pi, NDimensions )
        diffs[pi] = elem->pts[pi].coords[split_dim] - split_pos;

    UFor( pi, NDimensions )
    {
        if (diffs[pi] <= 0)
        {
            if (in_lo)
            {
                adjust_BBox (lobox, &elem->pts[pi]);
            }
            else
            {
                in_lo = true;
                copy_Point (&lobox->min, &elem->pts[pi]);
                copy_Point (&lobox->max, &elem->pts[pi]);
            }
        }

        if (diffs[pi] >= 0)
        {
            if (in_hi)
            {
                adjust_BBox (hibox, &elem->pts[pi]);
            }
            else
            {
                in_hi = true;
                copy_Point (&hibox->min, &elem->pts[pi]);
                copy_Point (&hibox->max, &elem->pts[pi]);
            }
        }
    }
        /* There should be a vertex on each side of the split.*/
    assert (in_lo);
    assert (in_hi);


        /* Loop over all pairs of vertices.*/
    UFor( pi, NDimensions-1 )
    {
        uint pj;
        const Point* a;
        a = &elem->pts[pi];

        for (pj = pi+1; pj < NDimensions; ++pj)
        {
            const Point* b;
            b = &elem->pts[pj];
                /* If the vertics are on opposite sides of the plane,
                 * find out where the line between them is clipped.
                 */
            if ((diffs[pi] != 0) && (diffs[pj] != 0) &&
                ((diffs[pi] < 0) == (diffs[pj] > 0)))
            {
                real m;
                Point isect;
                m = diffs[pi] / (a->coords[split_dim] - b->coords[split_dim]);

                    /* /isect = a + m*(b - a)/ */
                Op_Point_201200( &isect ,+, a ,m*, -, b , a );
                isect.coords[split_dim] = split_pos;

                adjust_BBox (lobox, &isect);
                adjust_BBox (hibox, &isect);
            }
        }
    }

    clip_BBox (lobox, lobox, box);
    clip_BBox (hibox, hibox, box);
}


static
    void
split_KDTreeGrid (KDTreeGrid* logrid, KDTreeGrid* higrid,
                  KDTreeGrid* grid,
                  uint split_dim, real split_pos, bool split_low,
                  const Simplex* elems)
{
    uint i, dim, nintls;
    uint loidx = 0, hiidx = 0;
    uint* lojumps;
    uint* hijumps;
    const real* bounds;

    assert (split_dim < NDimensions);
    nintls = 2 * grid->nelems;
    bounds = grid->coords[split_dim];

    init_KDTreeGrid (logrid, logrid->nelems);
    init_KDTreeGrid (higrid, higrid->nelems);

    lojumps = AllocT( uint, 2*nintls );
    hijumps = &lojumps[nintls];

    UFor( i, grid->nelems )
    {
        bool in_lo, in_hi;
        uint loti, hiti, elemidx;
        BBox lobox, hibox;
        loti = 2*i;
        hiti = 2*i+1;

        elemidx = grid->elemidcs[i];

        UFor( dim, NDimensions )
        {
            lobox.min.coords[dim] = grid->coords[dim][loti];
            lobox.max.coords[dim] = grid->coords[dim][hiti];
        }
        hibox = lobox;

        if (bounds[loti] == split_pos && bounds[hiti] == split_pos)
        {
            in_lo =  split_low;
            in_hi = !split_low;
        }
        else
        {
            in_lo = bounds[loti] < split_pos;
            in_hi = bounds[hiti] > split_pos;

            if (in_lo && in_hi && elems)
            {
                BBox box = lobox;
                splitclip_Simplex_BBox (&lobox, &hibox, &box,
                                        &elems[elemidx],
                                        split_dim, split_pos);
            }
        }

        lojumps[i] = Max_uint;
        if (in_lo)
        {
            lojumps[i] = loidx;
            assert (loidx < logrid->nelems);
            UFor( dim, NDimensions )
            {
                logrid->coords[dim][2*loidx+0] = lobox.min.coords[dim];
                logrid->coords[dim][2*loidx+1] = lobox.max.coords[dim];
            }
            logrid->elemidcs[loidx] = elemidx;
            loidx += 1;
        }

        hijumps[i] = Max_uint;
        if (in_hi)
        {
            hijumps[i] = hiidx;
            assert (hiidx < higrid->nelems);
            UFor( dim, NDimensions )
            {
                higrid->coords[dim][2*hiidx+0] = hibox.min.coords[dim];
                higrid->coords[dim][2*hiidx+1] = hibox.max.coords[dim];
            }
            higrid->elemidcs[hiidx] = elemidx;
            hiidx += 1;
        }
    }

    assert (loidx == logrid->nelems);
    assert (hiidx == higrid->nelems);

    UFor( dim, NDimensions )
    {
        loidx = 0;
        hiidx = 0;

        UFor( i, nintls )
        {
            uint ti, q, r;
            ti = grid->intls[dim][i];
            q = ti / 2;
            r = ti % 2;

            if (lojumps[q] != Max_uint)
                logrid->intls[dim][loidx++] = 2 * lojumps[q] + r;
            if (hijumps[q] != Max_uint)
                higrid->intls[dim][hiidx++] = 2 * hijumps[q] + r;
        }

        assert (loidx == 2*logrid->nelems);
        assert (hiidx == 2*higrid->nelems);

            /* TODO  Do something more clever!*/
        sort_indexed_reals (logrid->intls[dim], 0, 2*logrid->nelems,
                            logrid->coords[dim]);
        sort_indexed_reals (higrid->intls[dim], 0, 2*higrid->nelems,
                            higrid->coords[dim]);
    }

        /* Bounding box split allows equality, this KDTree build should not.*/
    assert (split_pos < grid->box.max.coords[split_dim]);
    assert (split_pos > grid->box.min.coords[split_dim]);

    split_BBox (&logrid->box, &higrid->box, &grid->box,
                       split_dim, split_pos);

    free (lojumps);
}


static
    real
kdtree_cost_fn (uint split_dim, real split_pos,
                uint nlo, uint nhi,
                const BBox* box)
{
    const real cost_it = 1;  /* Cost of intersection test.*/
    const real cost_tr = 2;  /* Cost of traversal.*/
    const real empty_bonus = .8;  /* Bonus for empty node!*/
    real cost;
    BBox lo_box, hi_box;
    real area;

    split_BBox (&lo_box, &hi_box, box, split_dim, split_pos);

    area = surface_area_BBox (box);
    if (area < Epsilon_real)  return Max_real;

    cost = (cost_tr
            + cost_it
            * (nlo * surface_area_BBox (&lo_box) +
               nhi * surface_area_BBox (&hi_box))
            / area);

    if (nlo == 0 || nhi == 0)
        cost *= empty_bonus;

    return cost;
}

static
    uint
determine_split (KDTreeGrid* logrid, KDTreeGrid* higrid, KDTreeGrid* grid,
                 const Simplex* elems)
{
    const real cost_it = 1;  /* Cost of intersection test.*/
    uint nelems, nintls;
    uint dim;
    real cost_split, cost_nosplit;
    uint nbelow = 0, nabove = 0, split_dim = NDimensions;
    real split_pos = 0;
    bool split_low = false;

    nelems = grid->nelems;
    nintls = 2*grid->nelems;

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
        lo_box = grid->box.min.coords[dim];
        hi_box = grid->box.max.coords[dim];
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

        /* printf ("cost:%f  cost_nosplit:%f\n", cost_split, cost_nosplit); */
    if (cost_split > cost_nosplit)  return NDimensions;

        /* At this point, the split is known.*/
    logrid->nelems = nbelow;
    higrid->nelems = nabove;
    split_KDTreeGrid (logrid, higrid, grid, split_dim,
                      split_pos, split_low, elems);

    return split_dim;
}

static
    void
build_KDTreeNode (KDTreeGrid* grid,
                  SList* nodelist, SList* elemidxlist,
                  uint depth,
                  const Simplex* elems)
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
        node->split_dim = determine_split (&logrid, &higrid, grid, elems);
    }

    if (node->split_dim == NDimensions)
    {
        KDTreeLeaf* leaf;
        leaf = &node->as.leaf;
        leaf->box = grid->box;

        leaf->nelems = grid->nelems;

        if (0 != leaf->nelems)
        {
            uint i;
            UFor( i, grid->nelems )
                app_uint_SList (elemidxlist, grid->elemidcs[i]);
        }
    }
    else
    {
        KDTreeInner* inner;
        inner = &node->as.inner;
        inner->split_pos = logrid.box.max.coords[node->split_dim];

#if 0
        {
            uint i;
            FILE* out = stderr;
            fprintf (out, "%*sdim:%u pos:%.1f  blw:%u abv:%u\n",
                     2*depth, "",
                     node->split_dim, inner->split_pos,
                     logrid->nintls/2, higrid->nintls/2);
            fprintf (out, "%*s", 2*depth+1, "");
            output_BBox (out, box);
            UFor( i, nelems )
            {
                fprintf (out, "\n%*s", 2*depth+1, "");
                output_Simplex (out, elems[i]);
            }
            fputc ('\n', out);
        }
#endif


            /* Perform tree split.*/
        {
            SList tmp_nodelist, tmp_elemidxlist;
#ifdef _OPENMP
#pragma omp parallel sections
#endif
            {

#ifdef _OPENMP
#pragma omp section
#endif
                {
                    build_KDTreeNode (&logrid,
                                      nodelist, elemidxlist,
                                      1+depth, elems);
                    cleanup_KDTreeGrid (&logrid);
                }

#ifdef _OPENMP
#pragma omp section
#endif
                {
                    init_SList (&tmp_nodelist);
                    init_SList (&tmp_elemidxlist);
                    build_KDTreeNode (&higrid,
                                      &tmp_nodelist, &tmp_elemidxlist,
                                      1+depth, elems);
                    cleanup_KDTreeGrid (&higrid);
                }
            }

            cat_SList (nodelist, &tmp_nodelist);
            cat_SList (elemidxlist, &tmp_elemidxlist);
        }
    }
}

static
    void
fixup_node_indices (KDTree* tree)
{
    bool ascending = false;
    uint node_idx = 0, parent_idx = 0;
    uint node_count = 0, elemidx_count = 0;
    KDTreeNode* nodes;

    nodes = tree->nodes;
    do
    {
        if (ascending)
        {
            KDTreeInner* inner;
            inner = &nodes[parent_idx].as.inner;
            if (inner->children[1] == Max_uint)
            {
                inner->children[1] = node_count;
                node_idx = node_count;
                ascending = false;
            }
            else
            {
                node_idx = parent_idx;
                parent_idx = inner->parent;
            }
        }
        else if (leaf_KDTreeNode (&nodes[node_idx]))
        {
            KDTreeLeaf* leaf;
            leaf = &nodes[node_idx].as.leaf;

            leaf->elemidcs = elemidx_count;
            elemidx_count += leaf->nelems;

            assert (node_idx == node_count);
            node_count += 1;
            ascending = true;
        }
        else
        {
            KDTreeInner* inner;
            inner = &nodes[node_idx].as.inner;

            inner->parent = parent_idx;
            parent_idx = node_idx;

            assert (node_idx == node_count);
            node_count += 1;
            node_idx = node_count;
            inner->children[0] = node_idx;
            inner->children[1] = Max_uint;
        }
    }
    while (node_idx != parent_idx);
    assert (node_count == tree->nnodes);
    assert (elemidx_count == tree->nelemidcs);
}

static
    bool
complete_KDTree (const KDTree* tree, const KDTreeGrid* grid)
{
    uint i;
    BitString* contains;
    bool pred = true;
    uint elemidx_capac = 0;

    UFor( i, grid->nelems )
        if (grid->elemidcs[i] >= elemidx_capac)
            elemidx_capac = grid->elemidcs[i] + 1;

    contains = alloc_BitString (elemidx_capac, true);
    UFor( i, grid->nelems )
        set0_BitString (contains, grid->elemidcs[i]);

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
                assert (idx < elemidx_capac);
                set1_BitString (contains, idx);
            }
        }
    }

    pred = all_BitString (elemidx_capac, contains);

    if (elemidx_capac > 0)  free_BitString (contains);
    return pred;
}

static int uintcmp (const void* a, const void* b)
{
    if (*(uint*)a < *(uint*)b)  return -1;
    if (*(uint*)a > *(uint*)b)  return  1;
    return 0;
}

    /* Build a tree of depth zero - everything in the root node.*/
    void
build_trivial_KDTree (KDTree* tree, uint nelems, const BBox* box)
{
    KDTreeLeaf* leaf;

    tree->nnodes = 1;
    tree->nodes = AllocT( KDTreeNode, tree->nnodes );

        /* Set the single leaf node to hold everything.*/
    tree->nodes[0].split_dim = NDimensions;
    leaf = &tree->nodes[0].as.leaf;
    leaf->box = *box;
    leaf->nelems = nelems;
    leaf->elemidcs = 0;

        /* Set element indices to be [0,..,n-1].*/
    tree->nelemidcs = nelems;
    tree->elemidcs = AllocT( uint, tree->nelemidcs );
    fill_minimal_unique (tree->elemidcs, tree->nelemidcs);
}

    void
build_KDTree (KDTree* tree, KDTreeGrid* grid, const Simplex* elems)
{
    uint i;
    real t0;
    SList nodelist, elemidxlist;

    t0 = monotime ();

#if MaxKDTreeDepth > 0
    UFor( i, NDimensions )
        sort_indexed_reals (grid->intls[i], 0, 2*grid->nelems, grid->coords[i]);
#endif

    init_SList (&nodelist);
    init_SList (&elemidxlist);
    build_KDTreeNode (grid, &nodelist, &elemidxlist, 0, elems);

    tree->nnodes = nodelist.nmembs;
    tree->nelemidcs = elemidxlist.nmembs;

    tree->nodes = AllocT( KDTreeNode, tree->nnodes );
    tree->elemidcs = AllocT( uint, tree->nelemidcs );

    unroll_SList (tree->nodes, &nodelist, sizeof (KDTreeNode));
    unroll_SList (tree->elemidcs, &elemidxlist, sizeof (uint));

    fixup_node_indices (tree);

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

    if (false)  printf ("kdtree build:%f\n", monotime () - t0);
    assert (complete_KDTree (tree, grid));
}
#endif  /* #ifndef __OPENCL_VERSION__ */


bool leaf_KDTreeNode (__global const KDTreeNode* node)
{
    return node->split_dim == NDimensions;
}


static
    uint
upnext_KDTreeNode (Point* entrance,
                   uint* ret_parent,
                   const Ray* ray,
                   const Point* invdirect,
                   real hit_mag,
                   uint node_idx,
                   __global const KDTreeNode* nodes)
{
    uint child_idx;
    uint split_dim;
    uint to_idx;

    {
        __global const KDTreeNode* node;
        __global const BBox* box;
        real mag;
        node = &nodes[node_idx];
        assert (leaf_KDTreeNode (node));
        box = &node->as.leaf.box;
        mag = hit_inner_BBox (entrance, &split_dim, box,
                              ray, invdirect);
        if (hit_mag < mag)
            return Max_uint;
    }

        /* Subtlety: Inclusive case opposite when descending tree.*/
    to_idx = ((ray->direct.coords[split_dim] < 0)
              ? 0 : 1);

    child_idx = node_idx;
    assert (ret_parent);
    node_idx = *ret_parent;

        /* Terminating condition:
         * Backtracked from root node => no possible next leaf.
         */
    while (child_idx != 0)
    {
        __global const KDTreeNode* node;
        __global const KDTreeInner* inner;

        node = &nodes[node_idx];

        assert (! leaf_KDTreeNode (node));
        inner = &node->as.inner;

            /* If not expected split dim or ray is pointing to
             * previously visited node, then keep backtracking.
             */
        if (split_dim == node->split_dim &&
            inner->children[to_idx] != child_idx)
        {
            *ret_parent = node_idx;
            return inner->children[to_idx];
        }

        child_idx = node_idx;
        node_idx = inner->parent;
    }
    return Max_uint;
}

static
    uint
descend_KDTreeNode (uint* ret_parent,
                    const Point* entrance,
                    uint node_idx,
                    __global const KDTreeNode* nodes)
{
    uint parent;
    __global const KDTreeNode* restrict node;
    node = &nodes[node_idx];

    parent = *ret_parent;
    while (! leaf_KDTreeNode (node))
    {
        __global const KDTreeInner* restrict inner;
        inner = &node->as.inner;
        parent = node_idx;

            /* Subtlety: Inclusive case here must be opposite of
             * inclusive case in upnext_KDTreeNode to avoid infinite
             * iteration on rays in the splitting plane's subspace.
             */
        node_idx = ((entrance->coords[node->split_dim] <= inner->split_pos)
                    ? inner->children[0]
                    : inner->children[1]);

        node = &nodes[node_idx];
    }

    *ret_parent = parent;
    return node_idx;
}

    uint
find_KDTreeNode (uint* ret_parent,
                 const Point* origin,
                 __global const KDTreeNode* nodes)
{
    return descend_KDTreeNode (ret_parent, origin, 0, nodes);
}


    uint
first_KDTreeNode (uint* ret_parent,
                  const Ray* restrict ray,
                  __global const KDTreeNode* restrict nodes,
                  const BBox* restrict box,
                  bool inside_box)
{
    uint node_idx = 0, parent = 0;

    if (inside_box)
    {
            /* Find the initial node.*/
        node_idx = find_KDTreeNode (&parent, &ray->origin, nodes);
        box = &nodes[node_idx].as.leaf.box;
        assert (inside_BBox (box, &ray->origin));
    }
    else
    {
        Point entrance;
        if (hit_outer_BBox (&entrance, box,
                            &ray->origin, &ray->direct))
            node_idx = descend_KDTreeNode (&parent, &entrance, 0, nodes);
        else
            node_idx = parent = Max_uint;
    }
    *ret_parent = parent;
    return node_idx;
}


    uint
next_KDTreeNode (uint* ret_parent,
                 const Ray* ray,
                 const Point* invdir,
                 real hit_mag,
                 uint node_idx,
                 __global const KDTreeNode* nodes)
{
    uint parent;
    Point entrance;
    parent = *ret_parent;

    node_idx = upnext_KDTreeNode (&entrance, &parent, ray, invdir,
                                  hit_mag,
                                  node_idx, nodes);

    if (node_idx == Max_uint)  return Max_uint;
    node_idx = descend_KDTreeNode (&parent, &entrance, node_idx, nodes);

    *ret_parent = parent;
    return node_idx;
}

