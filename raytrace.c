
#ifndef __OPENCL_VERSION__

#include "raytrace.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#ifdef _OPENMP
#include <omp.h>
#endif

static const bool KDTreeRayTrace = true;
static const bool BarycentricRayTrace = false;
    /* #define TrivialMpiRayTrace */

#ifdef TrivialMpiRayTrace
#include <mpi.h>
static void
balancer_triv_sync_mpi_RayImage (RayImage* image, uint nprocs);
static void
computer_triv_sync_mpi_RayImage (const RayImage* image,
                                 uint myrank, uint nprocs);
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

void init_RayImage (RayImage* image)
{
    image->hits = 0;
    image->mags = 0;
    image->pixels = 0;
    image->nrows = 0;
    image->ncols = 0;
    image->view_light = 0;
    image->shading_on = true;
    image->color_distance_on = false;
}

void resize_RayImage (RayImage* image)
{
    uint npixels;
    if (image->nrows == 0 || image->ncols == 0)  return;
    npixels = image->nrows * image->ncols;
    if (image->hits)  ResizeT( uint, image->hits, npixels );
    if (image->mags)  ResizeT( real, image->mags, npixels );
    if (image->pixels)  ResizeT( byte, image->pixels, 3 * npixels );
}

void cleanup_RayImage (RayImage* image)
{
    if (image->hits)  free (image->hits);
    if (image->mags)  free (image->mags);
    if (image->pixels)  free (image->pixels);
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


    void
fill_pixel (byte* ret_red, byte* ret_green, byte* ret_blue,
            real mag,
            const RayImage* image,
            const Point* dir,
            const BarySimplex* simplex)
{
    const uint nincs = 256;
    byte red, green, blue;

    if (!simplex)
    {
        *ret_red = 0;
        *ret_green = 0;
        *ret_blue = 0;
        return;
    }

    red = nincs-1;
    green = nincs-1;
    blue = nincs-1;

    if (image->color_distance_on && mag < image->view_light)
    {
        uint val;
        red = 0;
        green = 0;
        blue = 0;
            /* Distance color scale.*/
        val = (uint) (5 * nincs * (mag / image->view_light));
        if (val < nincs)
        {
            red = nincs - 1;
            green = val - 0 * nincs;
        }
        else if (val < 2 * nincs)
        {
            red = 2 * nincs - val - 1;
            green = nincs - 1;
        }
        else if (val < 3 * nincs)
        {
            green = nincs - 1;
            blue = val - 2 * nincs;
        }
        else if (val < 4 * nincs)
        {
            green = 4 * nincs - val - 1;
            blue = nincs - 1;
        }
        else if (val < 5 * nincs)
        {
            blue = nincs - 1;
            red = val - 4 * nincs;
        }
    }

    if (image->shading_on)
    {
        const real min_scale = 0.3;
        real scale;
        Point tmp;

        proj_Plane (&tmp, dir, &simplex->plane);
        scale = dot_Point (dir, &tmp);

        if (scale < 0)  scale = -scale;
        scale = 1 - scale;
        if (scale < 0)  scale = 0;
        scale = min_scale + (1 - min_scale) * scale;
        if (scale > 1)  scale = 1;
        
        red = (byte) (scale * red);
        green = (byte) (scale * green);
        blue = (byte) (scale * blue);
    }
                
    *ret_red = red;
    *ret_green = green;
    *ret_blue = blue;
}


static
    void
cast_ray_simple_barycentric (uint* restrict ret_hit,
                             real* restrict ret_mag,
                             const Point* restrict origin,
                             const Point* restrict dir,
                             uint nelems,
                             const BarySimplex* restrict elems)
{
    uint i;
    uint hit_idx;
    real hit_mag;

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
    void
rays_to_hits_fish (RayImage* image,
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
    uint nrows, ncols;
    real col_start, row_start;
    real col_delta, row_delta;

    nrows = image->nrows;
    ncols = image->ncols;

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
        uint* hitline = 0;
        real* magline = 0;
        byte* pixline = 0;

        if (image->hits)  hitline = &image->hits[row * ncols];
        if (image->mags)  magline = &image->mags[row * ncols];
        if (image->pixels)  pixline = &image->pixels[row * 3 * ncols];

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

            if (hitline)  hitline[col] = hit;
            if (magline)  magline[col] = mag;
            if (pixline)
            {
                byte red, green, blue;
                const BarySimplex* simplex = 0;
                if (hit < space->nelems)  simplex = &space->simplices[hit];
                fill_pixel (&red, &green, &blue,
                            mag, image, &dir, simplex);
                pixline[3*col+0] = red;
                pixline[3*col+1] = green;
                pixline[3*col+2] = blue;
            }
        }
    }
}


    void
rays_to_hits_fixed_plane (uint* hits, real* mags,
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


    void
rays_to_hits_parallel (RayImage* restrict image,
                       const RaySpace* restrict space,
                       const Point* restrict origin,
                       const PointXfrm* restrict view_basis,
                       real view_width)
{
    uint row;
    const uint row_dim = 0;
    const uint col_dim = 1;
    const uint dir_dim = DirDimension;
    uint nrows, ncols;
    real col_start, row_start;
    real col_delta, row_delta;
    const Point* restrict dir;
    const BoundingBox* box;

    nrows = image->nrows;
    ncols = image->ncols;

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
        uint* hitline = 0;
        real* magline = 0;
        byte* pixline = 0;

        if (image->hits)  hitline = &image->hits[row * ncols];
        if (image->mags)  magline = &image->mags[row * ncols];
        if (image->pixels)  pixline = &image->pixels[row * 3 * ncols];

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

            if (KDTreeRayTrace)
                cast_ray (&hit, &mag, &ray_origin, dir,
                          space->nelems, space->elems,
                          space->tree.elemidcs, space->tree.nodes,
                          &space->scene.box, inside_box);
            else if (BarycentricRayTrace)
                cast_ray_simple_barycentric (&hit, &mag, origin, dir,
                                             space->nelems, space->simplices);
            else
                cast_ray_simple (&hit, &mag, origin, dir,
                                 space->nelems, space->elems, view_basis);

            if (hitline)  hitline[col] = hit;
            if (magline)  magline[col] = mag;
            if (pixline)
            {
                byte red, green, blue;
                const BarySimplex* simplex = 0;
                if (hit < space->nelems)  simplex = &space->simplices[hit];
                fill_pixel (&red, &green, &blue,
                            mag, image, dir, simplex);
                pixline[3*col+0] = red;
                pixline[3*col+1] = green;
                pixline[3*col+2] = blue;
            }
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

    void
rays_to_hits (RayImage* restrict image,
              const RaySpace* restrict space,
              const Point* restrict origin,
              const PointXfrm* restrict view_basis,
              real view_angle)
{
    uint nprocs, myrank;
    uint row;
    uint nrows, ncols;
    bool inside_box;
    Point dir_start, row_delta, col_delta;
    const BoundingBox* restrict box;

    nrows = image->nrows;
    ncols = image->ncols;

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
        rays_to_hits_row (image, row, space, origin, view_basis,
                          &dir_start, &row_delta, &col_delta,
                          inside_box);
    }

#ifdef TrivialMpiRayTrace
    if (myrank == 0)  balancer_triv_sync_mpi_RayImage (image, nprocs);
    else              computer_triv_sync_mpi_RayImage (image, myrank, nprocs);
#endif
}


    void
rays_to_hits_row (RayImage* image, uint row,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const PointXfrm* restrict view_basis,
                  const Point* dir_start,
                  const Point* row_delta,
                  const Point* col_delta,
                  bool inside_box)
{
    uint col, ncols;
    Point partial_dir;
    uint* hitline = 0;
    real* magline = 0;
    byte* pixline = 0;

    ncols = image->ncols;

    if (image->hits)  hitline = &image->hits[row * ncols];
    if (image->mags)  magline = &image->mags[row * ncols];
    if (image->pixels)  pixline = &image->pixels[row * 3 * ncols];

    scale_Point (&partial_dir, row_delta, row);
    summ_Point (&partial_dir, &partial_dir, dir_start);

    UFor( col, ncols )
    {
        Point dir;
        uint hit;  real mag;


#if 0
        if (! (row == 10 && col == 10))
        {
            hitline[col] = space->nelems;
            continue;
        }
#endif

        scale_Point (&dir, col_delta, col);
        summ_Point (&dir, &dir, &partial_dir);
        normalize_Point (&dir, &dir);

        if (KDTreeRayTrace)
            cast_ray (&hit, &mag, origin, &dir,
                      space->nelems, space->elems,
                      space->tree.elemidcs, space->tree.nodes,
                      &space->scene.box, inside_box);
        else if (BarycentricRayTrace)
            cast_ray_simple_barycentric (&hit, &mag, origin, &dir,
                                         space->nelems, space->simplices);
        else
            cast_ray_simple (&hit, &mag, origin, &dir,
                             space->nelems, space->elems, view_basis);

        if (hitline)  hitline[col] = hit;
        if (magline)  magline[col] = mag;
        if (pixline)
        {
            byte red, green, blue;
            const BarySimplex* simplex = 0;
            if (hit < space->nelems)  simplex = &space->simplices[hit];
            fill_pixel (&red, &green, &blue,
                        mag, image, &dir, simplex);
            pixline[3*col+0] = red;
            pixline[3*col+1] = green;
            pixline[3*col+2] = blue;
        }

            /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
    }
}


#ifdef TrivialMpiRayTrace
    void
balancer_triv_sync_mpi_RayImage (RayImage* image, uint nprocs)
{
    uint i, nrows, ncols;
    nrows = image->nrows;
    ncols = image->ncols;
    for (i = 1; i < nprocs; ++i)
    {
        uint row;
        for (row = i; row < nrows; row += nprocs)
        {
            MPI_Status status;
            if (image->hits)
                MPI_Recv (&image->hits[row*ncols],
                          ncols * sizeof(uint), MPI_BYTE,
                          i, 1, MPI_COMM_WORLD, &status);
            if (image->mags)
                MPI_Recv (&image->mags[row*ncols],
                          ncols * sizeof(real), MPI_BYTE,
                          i, 1, MPI_COMM_WORLD, &status);
            if (image->pixels)
                MPI_Recv (&image->pixels[row*3*ncols],
                          ncols * sizeof(real), MPI_BYTE,
                          i, 1, MPI_COMM_WORLD, &status);
        }
    }
}

    void
computer_triv_sync_mpi_RayImage (const RayImage* image,
                                 uint myrank, uint nprocs)
{
    uint row, nrows, ncols;
    nrows = image->nrows;
    ncols = image->ncols;
    for (row = myrank; row < nrows; row += nprocs)
    {
        if (image->hits)
            MPI_Send (&image->hits[row*ncols],
                      ncols * sizeof(uint), MPI_BYTE,
                      0, 1, MPI_COMM_WORLD);
        if (image->mags)
            MPI_Send (&image->mags[row*ncols],
                      ncols * sizeof(real), MPI_BYTE,
                      0, 1, MPI_COMM_WORLD);
        if (image->pixels)
            MPI_Send (&image->pixels[row*3*ncols],
                      ncols * sizeof(real), MPI_BYTE,
                      0, 1, MPI_COMM_WORLD);
    }
}
#endif


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

