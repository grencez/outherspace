
#ifndef __OPENCL_VERSION__
#include "kptree.h"
#include "order.h"
#include "point.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

    void
init_KPTree (KPTree* tree)
{
    tree->nnodes = 0;
}

    void
cleanup_KPTree (KPTree* tree)
{
    if (tree->nnodes > 0)  free (tree->nodes);
}

    void
cleanup_KPTreeGrid (KPTreeGrid* grid)
{
    if (grid->npts > 0)
    {
        free (grid->indices);
        free (grid->coords[0]);
    }
}

static
    void
build_KPTreeNode (KPTree* tree, KPTreeGrid* grid,
                  uint nodeidx, uint p, uint s)
{
    uint i, n, mididx;
    uint split_dim;
    KPTreeNode* node;
    KPTreeGrid logrid, higrid;

    if (nodeidx >= tree->nnodes)  return;
    node = &tree->nodes[nodeidx];
    n = s - p;
    assert (n > 0);

        /* Choose a proper split index, store in /mididx/.*/
    mididx = p;
    if (n > 1)
    {
        uint depth, lo, hi;

            /* Max tree depth from this node.*/
        depth = log2_uint (n);
        assert (depth > 0);

            /* Count of leaf nodes at the max depth.
             *   Node count above depth: 2^depth - 1
             *   Node count total: n
             */
        lo = (n+1) - exp2_uint (depth);

            /* Max possible count of leaf nodes of left subtree.
             *   Max node count at depth: 2^depth
             * Since we are concerned only with left subtree,
             * split this in half.
             */
        hi = exp2_uint (depth - 1);

            /* Set /mididx/ as median index,
             * disregarding nodes at max depth.
             */
        mididx += (n-lo)/2;

            /* Increment /mididx/ such that its left
             * subtree is packed with nodes at max depth.
             */
        if (lo < hi)  mididx += lo;  /* Not completely filled, but packed.*/
        else          mididx += hi;  /* Completely filled (also packed).*/
    }

        /* Pick the widest dimension to split.*/
    split_dim = 0;
    if (n > 1)
    {
        real max_length = 0;

        UFor( i, NDimensions )
        {
            real len;
            len = (grid->box.max.coords[i]
                   - grid->box.min.coords[i]);
            if (len > max_length)
            {
                split_dim = i;
                max_length = len;
            }
        }
    }

        /* Find the split for /mididx/ between /p/ and /s/ indices.
         * /grid->indices/ is partitioned accordingly within
         * the range [p,..,s).
         */
    select_indexed_reals (grid->indices, grid->coords[split_dim],
                          p, mididx, s);
    assert (verify_select_indexed_reals (p, mididx, s,
                                         grid->indices,
                                         grid->coords[split_dim]));

        /* Assign this node's final values.*/
    node->dim = split_dim;
    node->idx = grid->indices[mididx];
    UFor( i, NDimensions )
        node->loc.coords[i] = grid->coords[i][node->idx];

        /* Setup for recursion.*/
    logrid.npts = grid->npts;
    higrid.npts = grid->npts;
    logrid.indices = grid->indices;
    higrid.indices = grid->indices;
    UFor( i, NDimensions )
    {
        logrid.coords[i] = grid->coords[i];
        higrid.coords[i] = grid->coords[i];
    }
        /* Only bother splitting the box if the lower nodes are not leaves.*/
    if (n > 3)
        split_BBox (&logrid.box, &higrid.box, &grid->box,
                    split_dim, node->loc.coords[split_dim]);

        /* Recurse for lo and hi sides of the split.*/
    build_KPTreeNode (tree, &logrid, 2*nodeidx+1, p, mididx);
    build_KPTreeNode (tree, &higrid, 2*nodeidx+2, mididx+1, s);
}

    void
build_KPTree (KPTree* tree, KPTreeGrid* grid)
{
    tree->nnodes = grid->npts;
    tree->nodes = AllocT( KPTreeNode, tree->nnodes );
    build_KPTreeNode (tree, grid, 0, 0, tree->nnodes);
}

#endif  /* #ifndef __OPENCL_VERSION__ */


    /* Descend the KPTree until we have an out-of-bounds (large) index
     * and return that. In effect, we find the index of some imaginary node
     * below node /i/, closest to /loc/, which has a real parent.
     * Algorithm-wise, this behavior is convenient, even though the returned
     * index is invalid.
     */
    uint
descend_KPTree (const KPTree* tree, const Point* loc, uint i)
{
    while (i < tree->nnodes)
    {
        const KPTreeNode* node;
        node = &tree->nodes[i];
        if (loc->coords[node->dim] <= node->loc.coords[node->dim])
            i = 2 * i + 1;
        else
            i = 2 * i + 2;
    }
    return i;
}

    /* Find the nearest Point, given a location.*/
    uint
nearest_neighbor_KPTree (const KPTree* tree, const Point* loc)
{
    uint i;
    uint best = Max_uint;
    real mag2 = Max_real;

    i = descend_KPTree (tree, loc, 0);

    while (i > 0)
    {
        bool fromlo;
        uint previ;
        real magtosplit;
        Point diff;
        const KPTreeNode* node;

        previ = i;
        i = (i - 1) / 2;  /* Ascend 1.*/
        node = &tree->nodes[i];

        magtosplit = node->loc.coords[node->dim] - loc->coords[node->dim];
        fromlo = magtosplit >= 0;

        if (fromlo != even_uint (previ) && magtosplit * magtosplit < mag2)
        {
            real tmpmag2;
            diff_Point (&diff, &node->loc, loc);
            tmpmag2 = dot_Point (&diff, &diff);
            if (tmpmag2 < mag2)
            {
                best = i;
                mag2 = tmpmag2;
            }

            if (fromlo)  i = 2 * i + 2;  /* From lo side -> go to hi side.*/
            else         i = 2 * i + 1;  /* From hi side -> go to lo side.*/
            i = descend_KPTree (tree, loc, i);
        }
    }

    return best;
}

    /* For each point found inside the bounding box,
     * the KPTreeNode index representing it is returned.
     *
     * Initiallly, /i/ is /Max_uint/.
     * Subsequently, /i/ is the previous return value.
     * Terminate when the return value is /Max_uint/.
     */
    uint
inside_BBox_KPTree (const KPTree* tree, const BBox* box, uint i)
{
    if (i < tree->nnodes)  i = 2 * i + 2;
    else                   i = 0;
    i = descend_KPTree (tree, &box->min, i);

    while (i > 0)
    {
        uint previ;

        previ = i;
        i = (i - 1) / 2;

        if (!even_uint (previ))
        {
            uint split_dim;
            real split_loc;
            const KPTreeNode* node;

            node = &tree->nodes[i];
            split_dim = node->dim;
            split_loc = node->loc.coords[split_dim];

            if (box->min.coords[split_dim] <= split_loc &&
                box->max.coords[split_dim] >= split_loc)
            {
                if (inside_BBox (box, &node->loc))  return i;

                i = 2 * i + 2;
                i = descend_KPTree (tree, &box->min, i);
            }
        }
    }
    return Max_uint;
}

