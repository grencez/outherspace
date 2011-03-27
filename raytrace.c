
#ifndef __OPENCL_VERSION__

#include "raytrace.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#define KDTreeRayTrace
    /* #define TrivialMpiRayTrace */
    /* #define BarycentricRayTrace */

#ifdef TrivialMpiRayTrace
#include <mpi.h>
#endif

void cleanup_RaySpace (RaySpace* space)
{
    cleanup_Scene (&space->scene);
    cleanup_KDTree (&space->tree);
    if (space->nelems > 0)
    {
        free (space->elems);
        free (space->simplices);
    }
}
#endif  /* #ifndef __OPENCL_VERSION__ */


void dir_from_MultiRayCastParams (Point* dir, uint row, uint col,
                                  const MultiRayCastParams* params)
{
    Point partial_dir;

    copy_Point (dir, &params->dir_start);

    partial_dir = params->dir_delta[0];
    scale_Point (&partial_dir, &partial_dir, row);
    summ_Point (dir, dir, &partial_dir);

    partial_dir = params->dir_delta[1];
    scale_Point (&partial_dir, &partial_dir, col);
    summ_Point (dir, dir, &partial_dir);
    normalize_Point (dir, dir);
}


#ifdef BarycentricRayTrace
static
    void
cast_ray_simple (uint* restrict ret_hit, real* restrict ret_mag,
                 const Point* restrict origin,
                 const Point* restrict dir,
                 uint nelems,
                 const BarySimplex* restrict elems,
                 const PointXfrm* view_basis)
{
    uint i;
    uint hit_idx;
    real hit_mag;

    (void) view_basis;

    hit_idx = nelems;
    hit_mag = Max_real;

    UFor( i, nelems )
    {
        real mag;
        if (hit_BarySimplex (&mag, origin, dir, &elems[i]))
        {
            if (mag < hit_mag)
            {
                hit_mag = mag;
                hit_idx = i;
            }
        }
    }
    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}
#else  /* #ifdef BarycentricRayTrace */
static
    void
cast_ray_simple (uint* restrict ret_hit, real* restrict ret_mag,
                 const Point* restrict origin,
                 const Point* restrict dir,
                 uint nelems,
                 const Triangle* restrict elems,
                 const PointXfrm* view_basis)
{
    uint i;
    uint hit_idx;
    real hit_mag;

    (void) view_basis;

    hit_idx = nelems;
    hit_mag = Max_real;

    UFor( i, nelems )
    {
        real mag;
        if (hit_Triangle (&mag, origin, dir, &elems[i]))
        {
            if (mag < hit_mag)
            {
                hit_mag = mag;
                hit_idx = i;
            }
        }
    }
    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}
#endif  /* #ifndef BarycentricRayTrace */

static
    void
cast_ray (uint* restrict ret_hit, real* restrict ret_mag,
          const Point* restrict origin,
          const Point* restrict dir,
          const uint nelems,
          __global const Triangle* restrict elems,
          __global const uint* restrict elemidcs,
          __global const KDTreeNode* restrict nodes,
          __global const BoundingBox* restrict box,
          bool inside_box)
{
    Point salo_entrance;
    uint node_idx, parent = 0;
    uint hit_idx;
    real hit_mag;
    Point* restrict entrance;

    entrance = &salo_entrance;

    hit_mag = Max_real;
    hit_idx = nelems;

    if (inside_box)
    {
            /* Find the initial node.*/
        node_idx = find_KDTreeNode (&parent, origin, nodes);
        box = &nodes[node_idx].as.leaf.box;
        assert (inside_BoundingBox (box, origin));
    }
    else
    {
        if (! hit_outer_BoundingBox (entrance, box, origin, dir))
        {
            *ret_hit = nelems;
            *ret_mag = Max_real;
            return;
        }
        node_idx = 0;
    }

    while (1)
    {
        __global const KDTreeNode* restrict node;
        node = &nodes[node_idx];

        if (leaf_KDTreeNode (node))
        {
            uint i;
            __global const KDTreeLeaf* restrict leaf;

            leaf = &node->as.leaf;
            box = &leaf->box;
                /* output_BoundingBox (stdout, box); */
                /* fputc ('\n', stdout); */

            UFor( i, leaf->nelems )
            {
                real mag;
                uint idx;
                const Triangle* restrict tri;
                idx = elemidcs[leaf->elemidcs + i];
#if __OPENCL_VERSION__
                const Triangle stri = elems[idx];
                tri = &stri;
#else
                tri = &elems[idx];
#endif
                    /* Triangle tri; */
                    /* elem_Scene (&tri, &space->scene, leaf->elems[i]); */

                if (hit_Triangle (&mag, origin, dir, tri))
                {
                    if (mag < hit_mag)
                    {
                        hit_mag = mag;
                        hit_idx = idx;
                    }
                }
            }

            if (hit_mag != Max_real)
            {
                Point hit;
                scale_Point (&hit, dir, hit_mag);
                summ_Point (&hit, &hit, origin);
                if (inside_BoundingBox (box, &hit))  break;
#if 0
                output_BoundingBox (stderr, box);
                fputs ("\n", stderr);
                output_Point (stderr, origin);
                fputs (" => ", stderr);
                output_Point (stderr, &hit);
                fputs ("\n", stderr);
#endif
            }

            node_idx = upnext_KDTreeNode (entrance, &parent,
                                          origin, dir, node_idx, nodes);
            if (node_idx == parent)  break;
        }
        else
        {
            __global const KDTreeInner* restrict inner;
            inner = &node->as.inner;
            parent = node_idx;

                /* Subtlety: Inclusive case here must be opposite of
                 * inclusive case in upnext_KDTreeNode to avoid infinite
                 * iteration on rays in the splitting plane's subspace.
                 */
            if (entrance->coords[node->split_dim] <= inner->split_pos)
                node_idx = inner->children[0];
            else
                node_idx = inner->children[1];
        }
    }
    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}

#ifndef __OPENCL_VERSION__
void rays_to_hits_fish (uint* hits, real* mags,
                        uint nrows, uint ncols,
                        const RaySpace* space,
                        const Point* origin,
                        const PointXfrm* view_basis,
                        real view_angle)
{
    uint row;
    bool inside_box;
    const uint row_dim = 0;
    const uint col_dim = 1;
    const uint dir_dim = DirDimension;
    real col_start, row_start;
    real col_delta, row_delta;

    row_start = - view_angle / 2;
    row_delta = view_angle / nrows;
    row_start += row_delta / 2;

    col_start = - view_angle / 2;
    col_delta = view_angle / ncols;
    col_start += col_delta / 2;

    inside_box = inside_BoundingBox (&space->scene.box, origin);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    UFor( row, nrows )
    {
        uint col;
        real row_angle;
        uint* hitline;
        real* magline;

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];

        row_angle = row_start + row_delta * row;

        UFor( col, ncols )
        {
            Point tdir, dir;
            uint hit; real mag;
            real col_angle;
            col_angle = col_start + col_delta * col;

            dir.coords[row_dim] = sin (row_angle);
            dir.coords[col_dim] = sin (col_angle);
            dir.coords[dir_dim] = cos (row_angle) + cos (col_angle);

            trxfrm_Point (&tdir, view_basis, &dir);


#if 0
            dir.coords[row_dim] = (tdir.coords[row_dim] * (1 + cos (row_angle))
                                   + tdir.coords[dir_dim] * sin (row_angle));

            dir.coords[col_dim] = (tdir.coords[col_dim] * (1 + cos (col_angle))
                                   + tdir.coords[dir_dim] * sin (col_angle));

            dir.coords[dir_dim] = (tdir.coords[dir_dim] * (cos (row_angle) + cos (col_angle))
                                   - tdir.coords[row_dim] * sin (row_angle)
                                   - tdir.coords[col_dim] * sin (col_angle));
#endif

            normalize_Point (&dir, &tdir);

            cast_ray (&hit, &mag, origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
            hitline[col] = hit;
            magline[col] = mag;
        }
    }
}


void rays_to_hits_fixed_plane (uint* hits, real* mags,
                               uint nrows, uint ncols,
                               const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint row_dim = 0;
    const uint col_dim = 1;
    const uint dir_dim = DirDimension;
    Point origin;
    real col_start, row_start;
    real col_delta, row_delta;
    const BoundingBox* box;

    box = &space->scene.box;

    row_start = space->scene.box.min_corner.coords[row_dim];
    row_delta = (box->max_corner.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = box->min_corner.coords[col_dim];
    col_delta = (box->max_corner.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    inside_box = (zpos > box->min_corner.coords[dir_dim] &&
                  zpos < box->max_corner.coords[dir_dim]);

    origin.coords[dir_dim] = zpos;
    origin.coords[row_dim] = 50;
    origin.coords[col_dim] = 50;
    
    inside_box = inside_BoundingBox (box, &origin);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    UFor( row, nrows )
    {
        uint col;
        uint* hitline;
        real* magline;

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];

        UFor( col, ncols )
        {
            Point dir;
            uint hit; real mag;

                /* if (! (row == 333 && col == 322))  continue; */

            dir.coords[dir_dim] = 0;
            dir.coords[row_dim] = row_start + row * row_delta;
            dir.coords[col_dim] = col_start + col * col_delta;

            diff_Point (&dir, &dir, &origin);
            normalize_Point (&dir, &dir);

            cast_ray (&hit, &mag,
                      &origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
            hitline[col] = hit;
            magline[col] = mag;

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}


void rays_to_hits_parallel (uint* hits, real* mags,
                            uint nrows, uint ncols,
                            const RaySpace* restrict space,
                            const Point* restrict origin,
                            const PointXfrm* restrict view_basis,
                            real view_width)
{
    uint row;
    const uint row_dim = 0;
    const uint col_dim = 1;
    const uint dir_dim = DirDimension;
    real col_start, row_start;
    real col_delta, row_delta;
    const Point* restrict dir;
    const BoundingBox* box;

    box = &space->scene.box;
    dir = &view_basis->pts[dir_dim];

    row_delta = view_width / nrows;
    row_start = (- view_width + row_delta) / 2;

    col_delta = view_width / ncols;
    col_start = (- view_width + col_delta) / 2;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    UFor( row, nrows )
    {
        uint col;
        Point partial_ray_origin;
        uint* hitline;
        real* magline;

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];

        scale_Point (&partial_ray_origin,
                     &view_basis->pts[row_dim],
                     row_start + row_delta * row);
        summ_Point (&partial_ray_origin, &partial_ray_origin, origin);

        UFor( col, ncols )
        {
            bool inside_box;
            Point ray_origin;
            uint hit; real mag;

            scale_Point (&ray_origin,
                         &view_basis->pts[col_dim],
                         col_start + col_delta * col);
            summ_Point (&ray_origin, &ray_origin, &partial_ray_origin);
            inside_box = inside_BoundingBox (box, &ray_origin);

#ifdef BarycentricRayTrace
            cast_ray_simple (&hit, &mag, origin, dir,
                             space->nelems, space->simplices, view_basis);
#else
#ifdef KDTreeRayTrace
            cast_ray (&hit, &mag, &ray_origin, dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
#else
            cast_ray_simple (&hit, &mag, origin, dir,
                             space->nelems, space->elems, view_basis);
#endif
#endif

            hitline[col] = hit;
            magline[col] = mag;
        }
    }
}


    void
setup_ray_pixel_deltas (Point* restrict dir_start,
                        Point* restrict row_delta,
                        Point* restrict col_delta,
                        uint nrows, uint ncols,
                        const PointXfrm* restrict view_basis,
                        real view_angle)
                    
{
    const uint row_dim = 0;
    const uint col_dim = 1;
    const uint dir_dim = DirDimension;
    Point dstart, rdelta, cdelta;
    real halflen;
    halflen = sin (view_angle / 2);

    zero_Point (&dstart);
    zero_Point (&rdelta);
    zero_Point (&cdelta);

    dstart.coords[row_dim] = - halflen;
    dstart.coords[col_dim] = - halflen;
    dstart.coords[dir_dim] = 1;

    rdelta.coords[row_dim] = -2 * dstart.coords[row_dim] / nrows;
    cdelta.coords[col_dim] = -2 * dstart.coords[col_dim] / ncols;

    dstart.coords[row_dim] -= dstart.coords[row_dim] / nrows;
    dstart.coords[col_dim] -= dstart.coords[col_dim] / ncols;

    trxfrm_Point (dir_start, view_basis, &dstart);
    trxfrm_Point (row_delta, view_basis, &rdelta);
    trxfrm_Point (col_delta, view_basis, &cdelta);
}

void rays_to_hits (uint* hits, real* mags,
                   uint nrows, uint ncols,
                   const RaySpace* restrict space,
                   const Point* restrict origin,
                   const PointXfrm* restrict view_basis,
                   real view_angle)
{
    uint nprocs, myrank;
    uint row;
    bool inside_box;
    Point dir_start, row_delta, col_delta;
    const BoundingBox* restrict box;

#ifdef TrivialMpiRayTrace
    MPI_Comm_size (MPI_COMM_WORLD, (int*) &nprocs);
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &myrank);
#else
    myrank = 0;
    nprocs = 1;
#endif

    box = &space->scene.box;
    setup_ray_pixel_deltas (&dir_start, &row_delta, &col_delta,
                            nrows, ncols,
                            view_basis, view_angle);

    inside_box = inside_BoundingBox (box, origin);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (row = myrank; row < nrows; row += nprocs )
    {
        uint col;
        uint* hitline;
        real* magline;
        Point partial_dir;

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];
        scale_Point (&partial_dir, &row_delta, row);
        summ_Point (&partial_dir, &partial_dir, &dir_start);

        UFor( col, ncols )
        {
            Point dir;
            uint hit; real mag;

#if 0
            if (! (row == 10 && col == 10))
            {
                hitline[col] = space->nelems;
                continue;
            }
#endif

            scale_Point (&dir, &col_delta, col);
            summ_Point (&dir, &dir, &partial_dir);
            normalize_Point (&dir, &dir);

#ifdef BarycentricRayTrace
            cast_ray_simple (&hit, &mag, origin, &dir,
                             space->nelems, space->simplices, view_basis);
#else
#ifdef KDTreeRayTrace
            cast_ray (&hit, &mag, origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
#else
            cast_ray_simple (&hit, &mag, origin, &dir,
                             space->nelems, space->elems, view_basis);
#endif
#endif
            hitline[col] = hit;
            magline[col] = mag;

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }

#ifdef TrivialMpiRayTrace
    if (myrank == 0)
    {
        uint i;
        for (i = 1; i < nprocs; ++i)
        {
            for (row = i; row < nrows; row += nprocs)
            {
                MPI_Status status;
                    /* fprintf (stderr, "Receiving hits from node %u for row %u!\n", i, row); */
                MPI_Recv (&hits[ncols*row], ncols * sizeof(uint), MPI_BYTE,
                          i, StdMsgTag, MPI_COMM_WORLD, &status);
                    /* fprintf (stderr, "Receiving mags from node %u for row %u!\n", i, row); */
                MPI_Recv (&mags[ncols*row], ncols * sizeof(real), MPI_BYTE,
                          i, StdMsgTag, MPI_COMM_WORLD, &status);
            }
        }
    }
    else {
        for (row = myrank; row < nrows; row += nprocs)
        {
                /* fprintf (stderr, "Node %u sending hits for row %u!\n", myrank, row); */
            MPI_Send (&hits[ncols*row], ncols * sizeof(uint), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
                /* fprintf (stderr, "Node %u sending mags for row %u!\n", myrank, row); */
            MPI_Send (&mags[ncols*row], ncols * sizeof(real), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
        }
    }
#endif  /* #ifdef TrivialMpiRayTrace */
}


void build_MultiRayCastParams (MultiRayCastParams* params,
                               uint nrows, uint ncols,
                               const RaySpace* space,
                               const Point* origin,
                               const PointXfrm* view_basis,
                               real view_angle)
{
    const uint row_dim = 0;
    const uint col_dim = 1;
    setup_ray_pixel_deltas (&params->dir_start,
                            &params->dir_delta[row_dim],
                            &params->dir_delta[col_dim],
                            nrows, ncols,
                            view_basis, view_angle);

    copy_BoundingBox (&params->box, &space->scene.box);
    params->nelems = space->nelems;
    params->npixels[row_dim] = nrows;
    params->npixels[col_dim] = ncols;
    params->inside_box = inside_BoundingBox (&params->box, origin);
}

#endif  /* #ifndef __OPENCL_VERSION__ */

