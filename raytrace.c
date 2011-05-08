
#ifndef __OPENCL_VERSION__

#include "raytrace.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#ifdef _OPENMP
#include <omp.h>
#endif

    /* Note: When setting this to false, set MaxKDTreeDepth = 0 in kdtree.c.*/
static const bool KDTreeRayTrace = true;
#if NDimensions == 3
static const bool BarycentricRayTrace = false;
#else
static const bool BarycentricRayTrace = true;
#endif
    /* #define TrivialMpiRayTrace */

#ifdef TrivialMpiRayTrace
#include <mpi.h>
static void
balancer_triv_sync_mpi_RayImage (RayImage* image, uint nprocs);
static void
computer_triv_sync_mpi_RayImage (const RayImage* image,
                                 uint myrank, uint nprocs);
#endif

void init_RaySpace (RaySpace* space)
{
    init_Scene (&space->scene);
    space->nelems = 0;
    space->tree.nnodes = 0;
    space->nobjects = 0;
    space->object_tree.nnodes = 0;
}

    void
init_filled_RaySpace (RaySpace* space)
{
    uint ei;
    const Scene* scene;
    scene = &space->scene;

    init_BoundingBox (&space->box, scene->nverts, scene->verts);

    space->nelems = scene->nelems;
    space->elems = AllocT( Simplex, space->nelems );

    UFor( ei, space->nelems )
    {
        uint pi;
        const SceneElement* elem;
        Simplex* tri;

        elem = &scene->elems[ei];
        tri = &space->elems[ei];

        UFor( pi, NDimensions )
        {
            uint vi;
            vi = elem->pts[pi];
            assert (vi < scene->nverts);
            copy_Point (&tri->pts[pi], &scene->verts[vi]);
        }
    }

    space->simplices = AllocT( BarySimplex, space->nelems );
    UFor( ei, space->nelems )
    {
        Simplex raw;
        bool good;
        simplex_Scene (&raw, scene, ei);
        good = init_BarySimplex (&space->simplices[ei], &raw);
        assert (good);
    }

    partition_RaySpace (space);
}

void cleanup_RaySpace (RaySpace* space)
{
    cleanup_Scene (&space->scene);
    cleanup_KDTree (&space->tree);
    if (space->nelems > 0)
    {
        free (space->elems);
        free (space->simplices);
    }
    if (space->nobjects > 0)
    {
        free (space->objects);
    }
}

void partition_RaySpace (RaySpace* space)
{
    KDTreeGrid grid;
    uint i;

    init_Scene_KDTreeGrid (&grid, &space->scene, &space->box);
    build_KDTree (&space->tree, &grid);
    cleanup_KDTreeGrid (&grid);

    UFor( i, space->nobjects )
        partition_RaySpace (&space->objects[i].space);
}

    void
init_Scene_KDTreeGrid (KDTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box)
{
    uint i, nelems;

    nelems = scene->nelems;
    assert (nelems > 0);

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
        uint pi, dim, ti;
        const SceneElement* elem;
        real coords[NDimensions][2];

        elem = &scene->elems[i];

        UFor( dim, NDimensions )
        {
            coords[dim][0] = Max_real;
            coords[dim][1] = Min_real;
        }

        UFor( pi, NDimensions )
        {
            const Point* p;
            p = &scene->verts[elem->pts[pi]];
            UFor( dim, NDimensions )
            {
                assert (inside_BoundingBox (box, p));
                if (p->coords[dim] < coords[dim][0])
                    coords[dim][0] = p->coords[dim];
                if (p->coords[dim] > coords[dim][1])
                    coords[dim][1] = p->coords[dim];
            }
        }

        ti = 2*i;
        UFor( dim, NDimensions )
        {
            grid->intls[dim][ti] = ti;
            grid->intls[dim][ti+1] = ti+1;
            grid->coords[dim][ti] = coords[dim][0];
            grid->coords[dim][ti+1] = coords[dim][1];
        }
    }
    copy_BoundingBox (&grid->box, box);
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

void downsample_RayImage (RayImage* image, uint inv)
{
    uint row, i_nrows, i_ncols, o_nrows, o_ncols;
    uint inv2;
    byte* o_pixline;
    byte* o_fracline;

    i_nrows = image->nrows;
    i_ncols = image->ncols;

    o_nrows = i_nrows / inv;
    o_ncols = i_ncols / inv;

    inv2 = inv * inv;
    
    o_pixline = AllocT( byte, 3 * (1 + o_ncols) );
    o_fracline = AllocT( byte, 3 * (1 + o_ncols) );

    memset (o_pixline, 0, 3 * o_ncols * sizeof(byte));
    memset (o_fracline, 0, 3 * o_ncols * sizeof(byte));

    UFor( row, i_nrows )
    {
        uint col;
        byte* i_pixline = 0;

        if (image->pixels)
            i_pixline = &image->pixels[row * 3 * i_ncols];

        UFor( col, i_ncols )
        {
            if (i_pixline)
            {
                uint i, o_off;
                o_off = 3 * (col / inv);
                UFor( i, 3 )
                {
                    uint x, y;
                    x = i_pixline[3*col+i];
                    y = (x % inv2) + o_fracline[o_off+i];
                    x = (x / inv2) + (y / inv2);
                    y = (y % inv2);
                    o_pixline[o_off+i] += x;
                    o_fracline[o_off+i] = y;
                }
            }
        }

        if ((row + 1) % inv == 0)
        {
            memcpy (&image->pixels[(row/inv) * 3 * o_ncols],
                    o_pixline,
                    3 * o_ncols * sizeof(byte));

            memset (o_pixline,  0, 3 * o_ncols * sizeof(byte));
            memset (o_fracline, 0, 3 * o_ncols * sizeof(byte));
        }
    }

    free (o_pixline);
    free (o_fracline);

    image->nrows = o_nrows;
    image->ncols = o_ncols;
    resize_RayImage (image);
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
            uint hit_idx,
            real mag,
            const RayImage* image,
            const Point* origin,
            const Point* dir,
            const BarySimplex* simplex,
            const Scene* scene)
{
    const bool shade_by_element = false;
    const bool color_by_element = false;
    const uint nincs = 256;
    byte rgb[3];
    const SceneElement* elem;
    const Material* material = 0;
    uint i;

    if (!simplex)
    {
        *ret_red = 0;
        *ret_green = 0;
        *ret_blue = 0;
        return;
    }

    elem = &scene->elems[hit_idx];
    if (elem->material != Max_uint)
        material = &scene->matls[elem->material];

    UFor( i, 3 )  rgb[i] = nincs-1;

    if (image->color_distance_on && mag < image->view_light)
    {
        uint val;
        UFor( i, 3 )  rgb[i] = 0;
            /* Distance color scale.*/
        val = (uint) (5 * nincs * (mag / image->view_light));
        if (val < nincs)
        {
            rgb[0] = nincs - 1;
            rgb[1] = val - 0 * nincs;
        }
        else if (val < 2 * nincs)
        {
            rgb[0] =  2 * nincs - val - 1;
            rgb[1] =  nincs - 1;
        }
        else if (val < 3 * nincs)
        {
            rgb[1] = nincs - 1;
            rgb[2] = val - 2 * nincs;
        }
        else if (val < 4 * nincs)
        {
            rgb[1] = 4 * nincs - val - 1;
            rgb[2] = nincs - 1;
        }
        else if (val < 5 * nincs)
        {
            rgb[2] = nincs - 1;
            rgb[0] = val - 4 * nincs;
        }
    }

    if (color_by_element)
    {
        uint color_diff, x, y;
        color_diff = 0xFFFFFF / scene->nelems;
        x = color_diff * (scene->nelems - hit_idx);
        y = 0;

        UFor( i, 3 )
        {
            uint j;
            UFor( j, 8 )
            {
                if (0 != (x & (1 << (i + 3*j))))
                    y |= (1 << (8*i + j));
            }
        }
        rgb[0] = (byte) ((y & 0xFF0000) >> 16);
        rgb[1] = (byte) ((y & 0x00FF00) >>  8);
        rgb[2] = (byte) ((y & 0x0000FF) >>  0);
    }
    else if (material && material->ambient_texture != Max_uint)
    {
            /* Texture mapping.*/
        const Texture* ambient_texture;
        Point isect, bpoint;
        BaryPoint texpoint;

        ambient_texture = &scene->txtrs[material->ambient_texture];

        scale_Point (&isect, dir, mag);
        summ_Point (&isect, &isect, origin);

        bpoint.coords[0] = 1;
        UFor( i, NDimensions-1 )
        {
            bpoint.coords[i+1] = distance_Plane (&simplex->barys[i], &isect);
            bpoint.coords[0] -= bpoint.coords[i+1];
            texpoint.coords[i] = 0;
        }

        UFor( i, 3 )
        {
            uint j;
            const BaryPoint* tmp;

            tmp = &scene->txpts[elem->txpts[i]];

            UFor( j, NDimensions-1 )
            {
                texpoint.coords[j] += bpoint.coords[i] * tmp->coords[j];
            }
        }

        map_Texture (rgb, ambient_texture, &texpoint);
    }

    if (shade_by_element)
    {
        real scale = 0;
        scale = 1 - (real) hit_idx / scene->nelems;
        UFor( i, 3 )
            rgb[i] *= scale;
    }
    else if (image->shading_on)
    {
        real scale;
        Point tmp;

        proj_Plane (&tmp, dir, &simplex->plane);
        scale = dot_Point (dir, &tmp);

        if (scale < 0)  scale = -scale;
        scale = 1 - scale;
        if (scale < 0)  scale = 0;

        UFor( i, 3 )
        {
            real ambient = 0.2;
            real diffuse = 1;
            real tscale;
            if (material)
            {
                ambient = material->ambient[i];
                diffuse = material->diffuse[i];
            }

            diffuse *= 1 - ambient;

            tscale = ambient + diffuse * scale;
            if (tscale > 1)  tscale = 1;

            rgb[i] = (byte) (tscale * rgb[i]);
        }
    }
                
    *ret_red = rgb[0];
    *ret_green = rgb[1];
    *ret_blue = rgb[2];
}


static
    bool
test_intersections (uint* ret_hit,
                    real* ret_mag,
                    const Point* restrict origin,
                    const Point* restrict dir,
                    uint nelemidcs,
                    __global const uint* restrict elemidcs,
                    __global const BarySimplex* restrict simplices,
                    __global const Simplex* restrict tris,
                    const BoundingBox* restrict box)
{
    uint i, hit_idx;
    real hit_mag;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;

    UFor( i, nelemidcs )
    {
        bool didhit;
        uint tmp_hit;
        real tmp_mag;

        if (BarycentricRayTrace)
        {
            tmp_hit = elemidcs[i];
            didhit = hit_BarySimplex (&tmp_mag, origin, dir,
                                      &simplices[tmp_hit]);
        }
        else 
        {
            const Simplex* restrict tri;
            tmp_hit = elemidcs[i];
#if __OPENCL_VERSION__
            const Simplex stri = tris[tmp_hit];
            tri = &stri;
#else
            tri = &tris[tmp_hit];
#endif
                /* Simplex tri; */
                /* elem_Scene (&tri, &space->scene, leaf->elems[i]); */
            didhit = hit_Simplex (&tmp_mag, origin, dir, tri);
        }

        if (didhit && tmp_mag < hit_mag)
        {
            hit_idx = tmp_hit;
            hit_mag = tmp_mag;
        }
    }


    *ret_hit = hit_idx;
    *ret_mag = hit_mag;

    if (hit_mag != Max_real)
    {
        Point isect;
        scale_Point (&isect, dir, hit_mag);
        summ_Point (&isect, &isect, origin);

        return inside_BoundingBox (box, &isect);
#if 0
        output_BoundingBox (stderr, box);
        fputs ("\n", stderr);
        output_Point (stderr, origin);
        fputs (" => ", stderr);
        output_Point (stderr, &isect);
        fputs ("\n", stderr);
#endif
    }
    return false;
}


static
    void
cast_ray (uint* restrict ret_hit, real* restrict ret_mag,
          const Point* restrict origin,
          const Point* restrict dir,
          const uint nelems,
          __global const uint* restrict elemidcs,
          __global const KDTreeNode* restrict nodes,
          __global const BarySimplex* restrict simplices,
          __global const Simplex* restrict tris,
          __global const BoundingBox* restrict box,
          bool inside_box)
{
    Point salo_entrance;
    uint node_idx, parent = 0;
    Point* restrict entrance;
    uint hit_idx;
    real hit_mag;

    entrance = &salo_entrance;

    hit_mag = Max_real;
    hit_idx = nelems;

    if (!KDTreeRayTrace)
    {
        test_intersections (&hit_idx, &hit_mag, origin, dir,
                            nelems, elemidcs,
                            simplices, tris, box);
        *ret_hit = hit_idx;
        *ret_mag = hit_mag;
        return;
    }

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

    do
    {
        __global const KDTreeLeaf* restrict leaf;
        bool hit_in_leaf;

        node_idx = descend_KDTreeNode (&parent, entrance, node_idx, nodes);

        leaf = &nodes[node_idx].as.leaf;
        hit_in_leaf =
            test_intersections (&hit_idx, &hit_mag, origin, dir,
                                leaf->nelems, &elemidcs[leaf->elemidcs],
                                simplices, tris, &leaf->box);
        if (hit_in_leaf)  break;

        node_idx = upnext_KDTreeNode (entrance, &parent,
                                      origin, dir, node_idx, nodes);
    }
    while (node_idx != parent);

    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}


static
    void
cast_recurse (uint* ret_hit,
              real* ret_mag,
              const RaySpace** ret_space,
              Point* ret_origin,
              Point* ret_dir,
              const RaySpace* restrict space,
              const PointXfrm* restrict view_basis,
              const Point* restrict origin,
              const Point* restrict dir,
              bool inside_box,
              uint ignore_object)
{
    uint hit_idx;
    real hit_mag;
    Point hit_origin, hit_dir;
    const RaySpace* hit_space;
    uint i;

    hit_space = space;

    cast_ray (&hit_idx, &hit_mag, origin, dir,
              space->nelems, space->tree.elemidcs,
              space->tree.nodes,
              space->simplices, space->elems,
              &space->box, inside_box);

    if (hit_idx < hit_space->nelems)
    {
        copy_Point (&hit_origin, origin);
        copy_Point (&hit_dir, dir);
    }

    UFor( i, space->nobjects )
    {
        Point diff, rel_origin, rel_dir;
        const RaySpaceObject* object;
        PointXfrm rel_basis;
        bool rel_inside_box;
            /* Returns from ray cast.*/
        uint tmp_hit;
        real tmp_mag;
        Point tmp_origin, tmp_dir;
        const RaySpace* tmp_space;

        if (i == ignore_object)  continue;

        object = &space->objects[i];

        diff_Point (&diff, origin, &object->centroid);
        xfrm_Point (&rel_origin, &object->orientation, &diff);
        summ_Point (&diff, &object->space.box.min_corner, 
                    &object->space.box.max_corner);
        scale_Point (&diff, &diff, 0.5);
        summ_Point (&rel_origin, &rel_origin, &diff);

        xfrm_Point (&rel_dir, &object->orientation, dir);

        rel_inside_box =
            inside_BoundingBox (&object->space.box, &rel_origin);

        xfrm_PointXfrm (&rel_basis, view_basis, &object->orientation);
        
        cast_recurse (&tmp_hit, &tmp_mag, &tmp_space,
                      &tmp_origin, &tmp_dir,
                      &object->space, &rel_basis,
                      &rel_origin, &rel_dir, rel_inside_box,
                      Max_uint);

        if (tmp_mag < hit_mag)
        {
            hit_idx = tmp_hit;
            hit_mag = tmp_mag;
            hit_space = tmp_space;
            copy_Point (&hit_origin, &tmp_origin);
            copy_Point (&hit_dir, &tmp_dir);
        }
    }

    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
    *ret_space = hit_space;
    if (hit_idx < hit_space->nelems)
    {
        copy_Point (ret_origin, &hit_origin);
        copy_Point (ret_dir, &hit_dir);
    }
}
              

static
    void
cast_record (uint* hitline,
             real* magline,
             byte* pixline,
             uint col,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const PointXfrm* restrict view_basis,
             const Point* restrict origin,
             const Point* restrict dir,
             bool inside_box)
{
    uint hit_idx;
    real hit_mag;
    const RaySpace* hit_space;
    Point hit_origin;
    Point hit_dir;

    cast_recurse (&hit_idx, &hit_mag, &hit_space,
                  &hit_origin, &hit_dir,
                  space, view_basis, origin, dir, inside_box,
                  Max_uint);

    if (hitline)  hitline[col] = hit_idx;
    if (magline)  magline[col] = hit_mag;
    if (pixline)
    {
        byte red, green, blue;
        const BarySimplex* simplex = 0;
        if (hit_idx < hit_space->nelems)
            simplex = &hit_space->simplices[hit_idx];
        fill_pixel (&red, &green, &blue,
                    hit_idx, hit_mag, image, &hit_origin, &hit_dir,
                    simplex, &hit_space->scene);
        pixline[3*col+0] = red;
        pixline[3*col+1] = green;
        pixline[3*col+2] = blue;
    }
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

    inside_box = inside_BoundingBox (&space->box, origin);

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

            cast_record (hitline, magline, pixline, col,
                         space, image, view_basis,
                         origin, &dir, inside_box);
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

    box = &space->box;

    row_start = space->box.min_corner.coords[row_dim];
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
                      space->nelems,
                      space->tree.elemidcs,
                      space->tree.nodes,
                      space->simplices, space->elems,
                      &space->box, inside_box);
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

    box = &space->box;
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

            scale_Point (&ray_origin,
                         &view_basis->pts[col_dim],
                         col_start + col_delta * col);
            summ_Point (&ray_origin, &ray_origin, &partial_ray_origin);
            inside_box = inside_BoundingBox (box, &ray_origin);

            cast_record (hitline, magline, pixline, col,
                         space, image, view_basis,
                         &ray_origin, dir, inside_box);
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

    box = &space->box;
    setup_ray_pixel_deltas (&dir_start, &row_delta, &col_delta,
                            nrows, ncols,
                            view_basis, view_angle);

    inside_box = inside_BoundingBox (box, origin);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (row = myrank; row < nrows; row += nprocs)
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

        cast_record (hitline, magline, pixline, col,
                     space, image, view_basis,
                     origin, &dir, inside_box);

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
                          3 * ncols, MPI_BYTE,
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
                      3 * ncols, MPI_BYTE,
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

    copy_BoundingBox (&params->box, &space->box);
    params->nelems = space->nelems;
    params->npixels[row_dim] = nrows;
    params->npixels[col_dim] = ncols;
    params->inside_box = inside_BoundingBox (&params->box, origin);
}

#endif  /* #ifndef __OPENCL_VERSION__ */

