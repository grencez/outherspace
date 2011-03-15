
#ifndef __OPENCL_VERSION__

#include "raytrace.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#ifdef _OPENMP
#include <omp.h>
#endif

    /* #define  TrivialMpiRayTrace */
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
    }
}
#endif  /* #ifndef __OPENCL_VERSION__ */


void dir_from_MultiRayCastParams (Point* dir, uint row, uint col,
                                  const MultiRayCastParams* params)
{
    Point partial_dir;

    copy_Point (dir, &params->dir_start);

    partial_dir = params->dir_delta[1];
    scale_Point (&partial_dir, &partial_dir, params->npixels[1] - row -1);
    summ_Point (dir, dir, &partial_dir);

    partial_dir = params->dir_delta[0];
    scale_Point (&partial_dir, &partial_dir, col);
    summ_Point (dir, dir, &partial_dir);
    normalize_Point (dir, dir);
}


#if 0
static
    bool
hit_tri (real* restrict dist,
         const Point* restrict origin, const Point* restrict dir,
         const Triangle* restrict elem)
{
    uint i, j, k;
    Triangle t;
    real trdots[NTrianglePoints];
    real tdots[NTrianglePoints];
    real dirdot;
    bool inbounds = false;

    dirdot = dot_Point (dir, dir);
    UFor( i, NTrianglePoints )
    {
        diff_Point (&t.pts[i], &elem->pts[i], origin);
        trdots[i] = dot_Point (dir, &t.pts[i]);
    }
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        tdots[i] = dirdot * dot_Point (&t.pts[j], &t.pts[k])
            - trdots[j] * trdots[k];
    }
    UFor( i, NTrianglePoints )
    {
        j = (1+i) % NTrianglePoints;
        k = (2+i) % NTrianglePoints;
        if (tdots[j] <= 0 && tdots[k] <= 0)
        {
            real x;
            x = dirdot * dot_Point (&t.pts[i], &t.pts[i])
                - trdots[i] * trdots[i];
            inbounds = (x * tdots[i] <= tdots[j] * tdots[k]);
            break;
        }
    }

    *dist = 50; /* TODO: this is no distance. */

#if 0
    if (inbounds)
    {
        zero_Point (hit);
        UFor( i, NTrianglePoints )
        {
            Point p;
            scale_Point (&p, dir, trdots[i]);
            summ_Point (hit, hit, &p);
        }

        UFor( i, NDimensions )
        {
            if (signum_real (hit->coords[i]) != signum_real (dir->coords[i]))
            {
                inbounds = false;
                break;
            }
        }

        if (inbounds)
        {
            scale_Point (hit, hit, 1 / (3 * dirdot));
            summ_Point (hit, hit, origin);
        }
    }
#endif
    return inbounds;
}

static
    bool
closer_hit (const Point* newhit, const Point* oldhit, const Point* dir)
{
    uint i;
    UFor( i, NDimensions )
    {
        tristate sign;
        sign = signum_real (dir->coords[i]);
        if (sign != 0)
            return sign == compare_real (oldhit->coords[i],
                                         newhit->coords[i]);
    }
    return false;
}

#elif 0


static
    void
cross_Point (Point* restrict dst,
             const Point* restrict a,
             const Point* restrict b)
{
    dst->coords[0] = a->coords[1] * b->coords[2] - a->coords[2] * b->coords[1];
    dst->coords[1] = a->coords[2] * b->coords[0] - a->coords[0] * b->coords[2];
    dst->coords[2] = a->coords[0] * b->coords[1] - a->coords[1] * b->coords[0];
}


static
    bool
hit_tri (real* restrict dist,
         const Point* restrict origin, const Point* restrict dir,
         const Triangle* restrict elem)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point edge1, edge2, tvec, pvec, qvec;
    real det, inv_det;
    real u, v;

    diff_Point (&tvec, origin, &elem->pts[0]);

    diff_Point (&edge2, &elem->pts[2], &elem->pts[0]);
    cross_Point (&pvec, dir, &edge2);

    diff_Point (&edge1, &elem->pts[1], &elem->pts[0]);
    u = dot_Point (&tvec, &pvec);
    det = dot_Point (&edge1, &pvec);

    if (det > epsilon)
    {
        if (u < 0 || u > det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (dir, &qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
        if (u > 0 || u < det)
            return false;

        cross_Point (&qvec, &tvec, &edge1);
        v = dot_Point (dir, &qvec);
        if (v > 0 || u + v < det)
            return false;
    }
    else
    {
        return false;
    }

    inv_det = 1 / det;
    *dist = dot_Point (&edge2, &qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return *dist >= 0;
}

#elif 1

static void cross3 (real dst[3], const real a[3], const real b[3])
{
    dst[0] = a[1] * b[2] - a[2] * b[1];
    dst[1] = a[2] * b[0] - a[0] * b[2];
    dst[2] = a[0] * b[1] - a[1] * b[0];
}


static real dot3 (const real a[3], const real b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


static
    bool
hit_tri (real* restrict dist,
         const Point* restrict origin, const Point* restrict kd_dir,
         const Triangle* restrict elem)
{
        /* const real epsilon = (real) 0.000001; */
    const real epsilon = 0;
    Point kd_edge1, kd_edge2, kd_tvec;
    real dir[3], edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    real det, inv_det;
    real u, v;

    diff_Point (&kd_edge1, &elem->pts[1], &elem->pts[0]);
    diff_Point (&kd_edge2, &elem->pts[2], &elem->pts[0]);
    diff_Point (&kd_tvec,  origin,        &elem->pts[0]);

    dir[0] = - dot_Point (kd_dir, kd_dir);
    dir[1] =   dot_Point (kd_dir, &kd_edge1);
    dir[2] =   dot_Point (kd_dir, &kd_edge2);

    edge1[0] = - dir[1];
    edge1[1] =   dot_Point (&kd_edge1, &kd_edge1);
    edge1[2] =   dot_Point (&kd_edge1, &kd_edge2);

    edge2[0] = - dir[2];
    edge2[1] =   edge1[2];
    edge2[2] =   dot_Point (&kd_edge2, &kd_edge2);

    tvec[0] = - dot_Point (kd_dir,    &kd_tvec);
    tvec[1] =   dot_Point (&kd_edge1, &kd_tvec);
    tvec[2] =   dot_Point (&kd_edge2, &kd_tvec);

    cross3 (pvec, dir, edge2);

    u = dot3 (tvec, pvec);
    det = dot3 (edge1, pvec);

    if (det > epsilon)
    {
        if (u < 0 || u > det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v < 0 || u + v > det)
            return false;

    }
    else if (det < -epsilon)
    {
        if (u > 0 || u < det)
            return false;

        cross3 (qvec, tvec, edge1);
        v = dot3 (dir, qvec);
        if (v > 0 || u + v < det)
            return false;
    }
    else
    {
        return false;
    }

    inv_det = 1 / det;
    *dist = dot3 (edge2, qvec) * inv_det;

        /* u *= inv_det; */
        /* v *= inv_det; */

    return *dist >= 0;
}
#endif

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
    real hit_mag;
    uint hit_idx;
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

                if (hit_tri (&mag, origin, dir, tri))
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
                        const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    Point origin, tdir;
    real col_start, row_start;
    real col_delta, row_delta;

    row_start = - M_PI / 3;
    row_delta = 2 * M_PI / (3 * nrows);
    row_start += row_delta / 2;

    col_start = - M_PI / 3;
    col_delta = 2 * M_PI / (3 * ncols);
    col_start += col_delta / 2;

    origin.coords[dir_dim] = zpos;
    origin.coords[row_dim] = 50;
    origin.coords[col_dim] = 50;
    
    tdir.coords[dir_dim] = 1;
    tdir.coords[row_dim] = 0;
    tdir.coords[col_dim] = 0;

    inside_box = inside_BoundingBox (&space->scene.box, &origin);

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

        row_angle = row_start + row_delta * (nrows - row -1);

        UFor( col, ncols )
        {
            Point dir;
            uint hit; real mag;
            real col_angle;
            col_angle = col_start + col_delta * col;

            dir.coords[row_dim] = (tdir.coords[row_dim] * (1 + cos (row_angle))
                                   + tdir.coords[dir_dim] * sin (row_angle));

            dir.coords[col_dim] = (tdir.coords[col_dim] * (1 + cos (col_angle))
                                   + tdir.coords[dir_dim] * sin (col_angle));

            dir.coords[dir_dim] = (tdir.coords[dir_dim] * (cos (row_angle) + cos (col_angle))
                                   - tdir.coords[row_dim] * sin (row_angle)
                                   - tdir.coords[col_dim] * sin (col_angle));

            normalize_Point (&dir, &dir);
            cast_ray (&hit, &mag, &origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
            hitline[col] = hit;
            magline[col] = mag;
        }
    }
}

void rays_to_hits_perspective (uint* hits, real* mags,
                               uint nrows, uint ncols,
                               const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
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
            dir.coords[row_dim] = row_start + (nrows - row -1) * row_delta;
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

void rays_to_hits_plane (uint* hits, real* mags,
                         uint nrows, uint ncols,
                         const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint dir_dim = 2;
    const uint row_dim = 1;
    const uint col_dim = 0;
    real col_start, row_start;
    real col_delta, row_delta;
    const BoundingBox* box;

    box = &space->scene.box;

    row_start = box->min_corner.coords[row_dim];
    row_delta = (box->max_corner.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = box->min_corner.coords[col_dim];
    col_delta = (box->max_corner.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    inside_box = (zpos > box->min_corner.coords[dir_dim] &&
                  zpos < box->max_corner.coords[dir_dim]);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    UFor( row, nrows )
    {
        uint col;
        Point origin, dir;
        uint* hitline;
        real* magline;

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];

        dir.coords[dir_dim] = 1;
        dir.coords[row_dim] = 0;
        dir.coords[col_dim] = 0;
        normalize_Point (&dir, &dir);

        origin.coords[dir_dim] = zpos;
        origin.coords[row_dim] = row_start + (nrows - row -1) * row_delta;

        UFor( col, ncols )
        {
            uint hit; real mag;
            origin.coords[col_dim] = col_start + col * col_delta;
            cast_ray (&hit, &mag, &origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
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
    const uint dir_dim = 2, row_dim = 1, col_dim = 0;
    Point dstart, rdelta, cdelta;
    real tcos;
    tcos = cos (view_angle / 2);

    zero_Point (&dstart);
    zero_Point (&rdelta);
    zero_Point (&cdelta);

    dstart.coords[dir_dim] = 1;
    dstart.coords[row_dim] = - tcos;
    dstart.coords[col_dim] = - tcos;

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
        scale_Point (&partial_dir, &row_delta, nrows - row -1);
        summ_Point (&partial_dir, &partial_dir, &dir_start);

        UFor( col, ncols )
        {
            Point dir;
            uint hit; real mag;

                /* if (! (row == 333 && col == 322))  continue; */

            scale_Point (&dir, &col_delta, col);
            summ_Point (&dir, &dir, &partial_dir);
            normalize_Point (&dir, &dir);

            cast_ray (&hit, &mag, origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
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
#endif
}


void build_MultiRayCastParams (MultiRayCastParams* params,
                               uint nrows, uint ncols,
                               const RaySpace* space,
                               const Point* origin,
                               const PointXfrm* view_basis,
                               real view_angle)
{
    const uint row_dim = 1, col_dim = 0;
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

