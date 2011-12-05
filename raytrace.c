
#ifndef __OPENCL_VERSION__

#include "bitstring.h"
#include "order.h"
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


#ifdef TrivialMpiRayTrace
#include <mpi.h>
static void
balancer_triv_sync_mpi_RayImage (RayImage* image,
                                 uint row_off, uint row_nul,
                                 uint nprocs);
static void
computer_triv_sync_mpi_RayImage (const RayImage* image,
                                 uint row_off, uint row_nul,
                                 uint myrank, uint nprocs);
#endif


static void
partition_ObjectRaySpace (ObjectRaySpace* space);
static void
partition_verts_ObjectRaySpace (ObjectRaySpace* space);
static void
init_Scene_KPTreeGrid (KPTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box);
static void
init_RaySpace_KDTreeGrid (KDTreeGrid* grid, const RaySpace* space);

static void
fill_pixel (real* ret_colors,
            uint hitidx, real mag, uint objidx,
            const RayImage* image,
            const Point* origin,
            const Point* dir,
            const RaySpace* space,
            uint nbounces);
static void
cast_colors (real* ret_colors,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const Point* restrict origin,
             const Point* restrict dir,
             const real* factors,
             uint nbounces);
static bool
cast_to_light (const RaySpace* restrict space,
               const Point* restrict origin,
               const Point* restrict dir,
               real magtolight);
static void
setup_ray_pixel_deltas_orthographic (Point* origin_start,
                                     Point* row_delta,
                                     Point* col_delta,
                                     uint nrows, uint ncols,
                                     const Point* origin,
                                     const PointXfrm* view_basis,
                                     real view_width);
static void
cast_row_orthographic (RayImage* restrict image,
                       uint row,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known);
static void
setup_ray_pixel_deltas_perspective (Point* dir_start,
                                    Point* row_delta,
                                    Point* col_delta,
                                    uint nrows, uint ncols,
                                    const PointXfrm* view_basis,
                                    real view_angle);
static void
cast_row_perspective (RayImage* image, uint row,
                      const RaySpace* restrict space,
                      const RayCastAPriori* restrict known);

    /* Include all packet tracing stuff here.
     * Keep dependence on this minimal.
     */
#ifdef PackOpsAvail
#include "raytrace-pack.c"
#endif


    void
init_RaySpace (RaySpace* space)
{
    init_ObjectRaySpace (&space->main);
    space->nobjects = 0;
    space->nlights = 0;
    init_KDTree (&space->object_tree);
    space->partition = true;
    zero_Point (&space->box.min);
    zero_Point (&space->box.max);
    space->skytxtr = 0;
}

    void
init_ObjectRaySpace (ObjectRaySpace* space)
{
    zero_Point (&space->centroid);
    identity_PointXfrm (&space->orientation);

    init_Scene (&space->scene);
    space->nelems = 0;
    init_KDTree (&space->tree);
    init_KPTree (&space->verttree);

    space->visible = true;
}

    void
init_PointLightSource (PointLightSource* light)
{
    zero_Point (&light->location);
    Op_s( real, NColors, light->intensity , 1 );
    light->diffuse = false;
    light->on = true;
}

    void
copy_PointLightSource (PointLightSource* dst, const PointLightSource* src)
{
    *dst = *src;
}

    void
init_filled_RaySpace (RaySpace* space)
{
    uint i;
    init_filled_ObjectRaySpace (&space->main);
    UFor( i, space->nobjects )
        init_filled_ObjectRaySpace (&space->objects[i]);
}

static
    void
update_internal_transformed_ObjectRaySpace (ObjectRaySpace* space)
{
    uint ei;
    const Scene* scene;
    scene = &space->scene;
    init_BoundingBox (&space->box, scene->nverts, scene->verts);

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
            vi = elem->verts[pi];
            assert (vi < scene->nverts);
            copy_Point (&tri->pts[pi], &scene->verts[vi]);
        }
    }

    UFor( ei, space->nelems )
    {
        Simplex raw;
        bool good;
        simplex_Scene (&raw, scene, ei);
        good = init_BarySimplex (&space->simplices[ei], &raw);
        if (!good)  printf ("ei:%u\n", ei);
        assert (good);
    }
}

    void
init_filled_ObjectRaySpace (ObjectRaySpace* object)
{
    object->nelems = object->scene.nelems;
    object->elems = AllocT( Simplex, object->nelems );
    object->simplices = AllocT( BarySimplex, object->nelems );

    update_internal_transformed_ObjectRaySpace (object);

    partition_ObjectRaySpace (object);
    partition_verts_ObjectRaySpace (object);
}

    void
init_trivial_ObjectRaySpace (ObjectRaySpace* object)
{
    object->nelems = object->scene.nelems;
    object->elems = AllocT( Simplex, object->nelems );
    object->simplices = AllocT( BarySimplex, object->nelems );

    update_internal_transformed_ObjectRaySpace (object);

    build_trivial_KDTree (&object->tree, object->nelems, &object->box);
}

    void
update_trivial_ObjectRaySpace (ObjectRaySpace* object)
{
    update_internal_transformed_ObjectRaySpace (object);
    copy_BoundingBox (&object->tree.nodes[0].as.leaf.box, &object->box);
}

    void
cleanup_RaySpace (RaySpace* space)
{
    uint i;
    cleanup_ObjectRaySpace (&space->main);
    UFor( i, space->nobjects )
        cleanup_ObjectRaySpace (&space->objects[i]);
    if (space->nobjects > 0)  free (space->objects);
    if (space->nlights > 0)  free (space->lights);
    cleanup_KDTree (&space->object_tree);
}

    void
cleanup_ObjectRaySpace (ObjectRaySpace* space)
{
    cleanup_Scene (&space->scene);
    cleanup_KDTree (&space->tree);
    cleanup_KPTree (&space->verttree);
    if (space->nelems > 0)
    {
        free (space->elems);
        free (space->simplices);
    }
}

    /* Partition the space containing dynamic objects.*/
    void
update_dynamic_RaySpace (RaySpace* space)
{
    if (space->partition && space->nobjects > 0)
    {
        KDTreeGrid grid;
        init_RaySpace_KDTreeGrid (&grid, space);
        copy_BoundingBox (&space->box, &grid.box);
            /* Since it's a regeneration, clean up the previous version.*/
        cleanup_KDTree (&space->object_tree);
        build_KDTree (&space->object_tree, &grid, 0);
            /* output_KDTreeGrid (stderr, &grid); */
            /* output_KDTree (stderr, &space->object_tree); */
        cleanup_KDTreeGrid (&grid);
    }
    else
    {
        space->partition = false;
        copy_BoundingBox (&space->box, &space->main.box);
    }
}

    void
partition_ObjectRaySpace (ObjectRaySpace* space)
{
    KDTreeGrid grid;
    init_Scene_KDTreeGrid (&grid, &space->scene, &space->box);
#if 1
    build_KDTree (&space->tree, &grid, space->elems);
#else
        /* Can use this code for development, less time spent building tree.*/
    build_KDTree (&space->tree, &grid, 0);
#endif
#if 0
        /* build_KDTree (&space->tree, &grid, 0); */
    printf ("nnodes:%u  nelemidcs:%u\n",
            space->tree.nnodes, space->tree.nelemidcs);
#endif
    cleanup_KDTreeGrid (&grid);
}

    void
partition_verts_ObjectRaySpace (ObjectRaySpace* space)
{
    KPTreeGrid grid;
    init_Scene_KPTreeGrid (&grid, &space->scene, &space->box);
    build_KPTree (&space->verttree, &grid);
    cleanup_KPTreeGrid (&grid);
}

    void
init_Scene_KDTreeGrid (KDTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box)
{
    uint i, nelems;

    nelems = scene->nelems;
    assert (nelems > 0);

    init_KDTreeGrid( grid, nelems );
    fill_minimal_unique (grid->elemidcs, grid->nelems);
    UFor( i, NDimensions )
        fill_minimal_unique (grid->intls[i], 2*nelems);

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
            p = &scene->verts[elem->verts[pi]];
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
            grid->coords[dim][ti] = coords[dim][0];
            grid->coords[dim][ti+1] = coords[dim][1];
        }
    }
    copy_BoundingBox (&grid->box, box);
}

    void
init_Scene_KPTreeGrid (KPTreeGrid* grid, const Scene* scene,
                       const BoundingBox* box)
{
    uint i;

    grid->npts = scene->nverts;
    grid->indices = AllocT( uint, grid->npts );
    grid->coords[0] = AllocT( real, NDimensions * grid->npts );
    copy_BoundingBox (&grid->box, box);

    UFor( i, NDimensions-1 )
        grid->coords[i+1] = &grid->coords[i][grid->npts];

    UFor( i, grid->npts )
    {
        uint dim;
        const Point* p;

        grid->indices[i] = i;
        p = &scene->verts[i];

        UFor( dim, NDimensions )
            grid->coords[dim][i] = p->coords[dim];
    }
}

    void
init_RaySpace_KDTreeGrid (KDTreeGrid* grid, const RaySpace* space)
{
    uint i;
    uint nvisible = 0;

    init_KDTreeGrid (grid, 1 + space->nobjects);

    UFor( i, NDimensions )
    {
        fill_minimal_unique (grid->intls[i], 2*grid->nelems);
        grid->box.max.coords[i] = Min_real;
        grid->box.min.coords[i] = Max_real;
    }

    UFor( i, space->nobjects+1 )
    {
        uint dim, ti;
        const ObjectRaySpace* object;
        BoundingBox box;

        if (i < space->nobjects)  object = &space->objects[i];
        else                      object = &space->main;

        if (!object->visible)  continue;

        ti = 2 * nvisible;
        
        if (i < space->nobjects)
            trxfrm_BoundingBox (&box,
                                &object->orientation,
                                &object->box,
                                &object->centroid);
        else
            copy_BoundingBox (&box, &object->box);

        include_BoundingBox (&grid->box, &grid->box, &box);

        grid->elemidcs[nvisible] = i;
        UFor( dim, NDimensions )
        {
            real lo, hi;
            lo = box.min.coords[dim];
            hi = box.max.coords[dim];
            grid->coords[dim][ti] = lo;
            grid->coords[dim][ti+1] = hi;
        }
        nvisible += 1;
    }
    shrink_KDTreeGrid (grid, nvisible);
}


void init_RayImage (RayImage* image)
{
    uint i;
    image->hits = 0;
    image->mags = 0;
    image->pixels = 0;
    image->nrows = 0;
    image->stride = 0;
    image->ncols = 0;
    image->hifov = 2 * M_PI / 3;
    image->perspective = true;
    UFor( i, NColors )
        image->ambient[i] = 0.2;
    image->view_light = 0;
    image->shading_on = true;
    image->color_distance_on = false;
    image->nbounces_max = 2;
}

void resize_RayImage (RayImage* image)
{
#ifdef PackOpsAvail
    const uint align = RayPacketDimSz;
#else
    const uint align = 4;
#endif
    uint npixels;
    if (image->nrows == 0 || image->ncols == 0)  return;
    image->stride = align * ceil_uint (image->ncols, align);
    npixels = image->nrows * image->stride;
    if (image->hits)  ResizeT( uint, image->hits, npixels );
    if (image->mags)  ResizeT( real, image->mags, npixels );
    if (image->pixels)  ResizeT( byte, image->pixels, 3 * npixels );
}

void restride_RayImage (RayImage* image)
{
    uint i;
        /* This should be set to the desired stride,
         * initialized by a resize.
         */
    assert (image->stride >= image->ncols);
    if (image->stride == image->ncols)  return;
    UFor( i, image->nrows )
    {
        uint row, dst_idx, src_idx;
        row = image->nrows - i - 1;
        dst_idx = row * image->stride;
        src_idx = row * image->ncols;

        if (image->hits)
            memmove (&image->hits[dst_idx], &image->hits[src_idx],
                     image->ncols * sizeof (uint));
        if (image->mags)
            memmove (&image->mags[dst_idx], &image->mags[src_idx],
                     image->ncols * sizeof (real));
        if (image->pixels)
            memmove (&image->pixels[3 * dst_idx], &image->pixels[3 * src_idx],
                     image->ncols * 3 * sizeof (byte));
    }
}

    /** Remove the stride from a RayImage to make
     * /image->cols == image->stride/.
     **/
void unstride_RayImage (RayImage* image)
{
    uint row;
    if (image->ncols == image->stride)  return;
    UFor( row, image->nrows )
    {
        uint dst_idx, src_idx;
        dst_idx = row * image->ncols;
        src_idx = row * image->stride;

        if (image->hits)
            memmove (&image->hits[dst_idx], &image->hits[src_idx],
                     image->ncols * sizeof (uint));
        if (image->mags)
            memmove (&image->mags[dst_idx], &image->mags[src_idx],
                     image->ncols * sizeof (real));
        if (image->pixels)
            memmove (&image->pixels[3 * dst_idx], &image->pixels[3 * src_idx],
                     image->ncols * 3 * sizeof (byte));
    }
    image->stride = image->ncols;
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

    if (image->pixels)
    {
        UFor( row, i_nrows )
        {
            uint col;
            byte* i_pixline;

            i_pixline = &image->pixels[row * 3 * image->stride];

            UFor( col, i_ncols )
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

            if ((row + 1) % inv == 0)
            {
                uint n;
                n = 3 * o_ncols;
                CopyT( byte, &image->pixels[(row/inv) * n], o_pixline, 0, n );
                memset (o_pixline,  0, n * sizeof(byte));
                memset (o_fracline, 0, n * sizeof(byte));
            }
        }
    }

    UFor( row, o_nrows )
    {
        uint col;
        UFor( col, o_ncols )
        {
            uint dst_idx, src_idx;
            dst_idx =        row * o_ncols       + col;
            src_idx = inv * (row * image->stride + col);
            if (image->hits)
                image->hits[dst_idx] = image->hits[src_idx];
            if (image->mags)
                image->mags[dst_idx] = image->mags[src_idx];
        }
    }

    free (o_pixline);
    free (o_fracline);

    image->nrows = o_nrows;
    image->ncols = o_ncols;
    resize_RayImage (image);
    image->stride = image->ncols;
}

void cleanup_RayImage (RayImage* image)
{
    if (image->hits)  free (image->hits);
    if (image->mags)  free (image->mags);
    if (image->pixels)  free (image->pixels);
}
#endif  /* #ifndef __OPENCL_VERSION__ */


    void
map_vertex_normal (Point* normal,
                   const Point* vnmls,
                   const SceneElement* elem,
                   const Point* bpoint)
{
    uint i;
    zero_Point (normal);
    UFor( i, NDimensions )
    {
        assert (elem->vnmls[i] != Max_uint);
        Op_Point_2010( normal
                       ,+, normal
                       ,   bpoint->coords[i]*, &vnmls[elem->vnmls[i]] );
    }
    normalize_Point (normal, normal);
}

    /* TODO: Make this function useful.*/
    void
map_isect_height (Point* ret_isect,
                  const Point* isect,
                  const Point* bpoint,
                  const SceneElement* elem,
                  const Point* verts,
                  const Point* vnmls)
{
    uint i;
    Point tmp_isect;
    zero_Point (&tmp_isect);
    UFor( i, NDimensions )
    {
        Point tmp;
        Plane plane;
        assert (elem->vnmls[i] != Max_uint);
        copy_Point (&plane.normal, &vnmls[elem->vnmls[i]]);
        plane.offset = - dot_Point (&plane.normal, &verts[elem->verts[i]]);
        proj_Plane (&tmp, isect, &plane);
        Op_Point_2010( &tmp_isect
                       ,+, &tmp_isect
                       ,   bpoint->coords[i]*, &tmp);
    }
    copy_Point (ret_isect, &tmp_isect);
}

static
    const ObjectRaySpace*
ray_to_ObjectRaySpace (Point* ret_origin,
                       Point* ret_dir,
                       const Point* origin,
                       const Point* dir,
                       const RaySpace* space,
                       uint objidx)
{
    const ObjectRaySpace* object;
    assert (objidx <= space->nobjects);
    if (objidx < space->nobjects)
    {
        object = &space->objects[objidx];
        ray_to_basis (ret_origin, ret_dir,
                      &object->orientation,
                      origin, dir,
                      &object->centroid);
    }
    else
    {
        object = &space->main;
        copy_Point (ret_origin, origin);
        copy_Point (ret_dir, dir);
    }
    return object;
}

static
    void
refraction_ray (Point* dst, const Point* dir, const Point* normal,
                real r, bool entering, real cos_normal)
{
    Point a, b;
    real d;
    if (r == 1)
    {
        copy_Point (dst, dir);
        return;
    }
        /*        dir
         *  A = -------
         *      ||N*D||
         */
    scale_Point (&a, dir, 1 / cos_normal);
        /* B = A + N */
    summ_Point (&b, &a, normal);
        /*                    1
         *  d = ------------------------------------
         *      sqrt(((r1/r2)^2 * ||A||^2) - ||B||^2)
         */
    if (!entering)  r = 1 / r;
    d = r * r * dot_Point (&a, &a) - dot_Point (&b, &b);
    d = 1 / sqrt (d);
        /* dst = d*A + (1-d)(-N) */
    scale_Point (&a, &a, d);
    scale_Point (&b, normal, d-1);
    summ_Point (dst, &a, &b);
}

static
    uint
splitting_plane_count (const Point* origin, const Point* direct, real mag,
                       const KDTree* tree, const BoundingBox* box)
{
    uint count = 0;
    uint node_idx, parent, destin_nodeidx;
    Point destin;
    Point invdirect;
    Ray ray;
    bool inside_box;

    copy_Point (&ray.origin, origin);
    copy_Point (&ray.direct, direct);

    Op_Point_2010( &destin ,+, origin ,mag*, direct );
    if (inside_BoundingBox (box, &destin))
        destin_nodeidx = find_KDTreeNode (&parent, &destin,
                                          tree->nodes);
    else
        destin_nodeidx = Max_uint;
    
#if 0
        /* Return the number of elements in the hit node.*/
    return ((destin_nodeidx == Max_uint) ? 0 :
            tree->nodes[destin_nodeidx].as.leaf.nelems);
#endif

    inside_box = inside_BoundingBox (box, origin);

    invmul_Point (&invdirect, direct);
    node_idx = first_KDTreeNode (&parent, &ray,
                                 tree->nodes,
                                 box, inside_box);

    if (node_idx != Max_uint && inside_box)
        count += 1;

    while (node_idx != Max_uint && node_idx != destin_nodeidx)
    {
        count += 1;
        node_idx = next_KDTreeNode (&parent, &ray, &invdirect,
                                    Max_real,
                                    node_idx, tree->nodes);
    }
    return count;
}

    void
fill_pixel (real* ret_colors,
            uint hitidx, real mag, uint objidx,
            const RayImage* image,
            const Point* origin,
            const Point* dir,
            const RaySpace* space,
            uint nbounces)
{
    const real offset_factor = 1e2 * Epsilon_real;
    real offset;
    const bool shade_by_element = false;
    const bool color_by_element = false;
    const bool compute_bary_coords = true;
    const bool show_splitting_planes = false;
    bool miss_effects = false;
    real colors[NColors];
    const BarySimplex* simplex;
    const ObjectRaySpace* object;
    Point rel_origin, rel_dir;
    const Scene* scene;
    const SceneElement* elem;
    const Material* material = 0;
    bool hit_front;
    real cos_normal;
    Point bpoint, normal;
    uint i;

    if (show_splitting_planes)
        miss_effects = true;

    if (objidx > space->nobjects)
    {
        if (space->skytxtr < space->main.scene.ntxtrs)
        {
            map_sky_Texture (ret_colors,
                             &space->main.scene.txtrs[space->skytxtr],
                             dir);
        }
        else
        {
            UFor( i, NColors )  ret_colors[i] = 0;
        }

        if (!miss_effects)  return;
        UFor( i, NColors )  colors[i] = ret_colors[i];
    }
    else
    {
        UFor( i, NColors )  colors[i] = 1;
    }

    if (show_splitting_planes)
    {
        const real frac = .1;
        real red;
        uint nplanes;

        nplanes = splitting_plane_count (origin, dir, mag,
#if 1
                                         &space->main.tree,
                                         &space->main.box
#else
                                         &space->object_tree,
                                         &space->box
#endif
                                        );

        red = 1;
        UFor( i, nplanes )
            red = (1 - frac) * red;

        UFor( i, NColors )
        {
            if (i == 0)
                colors[i] = clamp_real (1 - red * (1 - colors[i]), 0, 1);
            else
                colors[i] = clamp_real (red * colors[i], 0, 1);
        }
    }

    if (objidx > space->nobjects)
    {
        UFor( i, NColors )  ret_colors[i] += colors[i];
        return;
    }


        /* Use 1 + the L1 (taxicab) norm to scale the offset of the origin of
         * subsequent rays (reflection, to-light, etc.) from the computed
         * intersection point so those rays do not hit this object.
         * The 1 is added in case the L1 norm is less than 1.
         * In the end, a very small factor relating to floating point precision
         * is multiplied on, so we should get a small offset magnitude.
         */
    offset = 1;
    UFor( i, NDimensions )
    {
        if (origin->coords[i] < 0)  offset -= origin->coords[i];
        else                        offset += origin->coords[i];
    }
    offset *= offset_factor;

    object = ray_to_ObjectRaySpace (&rel_origin, &rel_dir,
                                    origin, dir, space, objidx);

    simplex = &object->simplices[hitidx];
    hit_front = (0 >= dot_Point (&rel_dir, &simplex->plane.normal));

    scene = &object->scene;

    elem = &scene->elems[hitidx];
    if (elem->material != Max_uint)
        material = &scene->matls[elem->material];

    if (image->color_distance_on && mag < image->view_light)
    {
        real val;
            /* Distance color scale.*/
        UFor( i, NColors )  colors[i] = 0;
        val = 2 * NColors * mag / image->view_light;
        UFor( i, 2*NColors )
        {
            if (val < i+1)
            {
                uint idx1, idx2;
                idx1 = i / 2;
                idx2 = (idx1 + 1) % NColors;
                if (even_uint (i))
                {
                    colors[idx1] = 1;
                    colors[idx2] = val - i;
                }
                else
                {
                    colors[idx1] = i+1 - val;
                    colors[idx2] = 1;
                }
                break;
            }
        }
    }

    if (compute_bary_coords)
    {
        Point rel_isect;
        Op_Point_2010( &rel_isect ,+, &rel_origin ,mag*, &rel_dir );
        barycentric_Point (&bpoint, &rel_isect, simplex);
    }

        /* Get the normal.*/
    if (compute_bary_coords && 0 < scene->nvnmls)
        map_vertex_normal (&normal, scene->vnmls, elem, &bpoint);
    else
        normalize_Point (&normal, &simplex->plane.normal);
        /* Rotate it when necessary.*/
    if (objidx != space->nobjects)
        trxfrm_Point (&normal, &object->orientation, &normal);

        /* Assure the normal is in our direction.*/
    cos_normal = dot_Point (dir, &normal);
    if (hit_front)
        cos_normal = - cos_normal;
    else
        negate_Point (&normal, &normal);

    if (color_by_element)
    {
        uint color_diff, x, y;
        const uint nincs = 256;
        assert (NColors == 3);
        color_diff = 0xFFFFFF / scene->nelems;
        x = color_diff * (scene->nelems - hitidx);
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
        colors[0] *= (real) ((y & 0xFF0000) >> 16) / (nincs-1);
        colors[1] *= (real) ((y & 0x00FF00) >>  8) / (nincs-1);
        colors[2] *= (real) ((y & 0x0000FF) >>  0) / (nincs-1);
    }
    else if (compute_bary_coords &&
             material && material->ambient_texture != Max_uint)
    {
            /* Texture mapping.*/
        const Texture* ambient_texture;
        BaryPoint texpoint;
        assert (NColors == 3);
        ambient_texture = &scene->txtrs[material->ambient_texture];
        UFor( i, NDimensions-1 )
            texpoint.coords[i] = 0;

        UFor( i, 3 )
        {
            uint j;
            const BaryPoint* tmp;
            tmp = &scene->txpts[elem->txpts[i]];

            UFor( j, NDimensions-1 )
                texpoint.coords[j] += bpoint.coords[i] * tmp->coords[j];
        }
        map_Texture (colors, ambient_texture, &texpoint);
    }

    if (shade_by_element)
    {
        real scale = 0;
        scale = 1 - (real) hitidx / scene->nelems;
        UFor( i, NColors )
            colors[i] *= scale;
    }
    else if (image->shading_on)
    {
        bool transparent = false;
        bool reflective = false;
        Point isect, refldir;
        real dscale[NColors], sscale[NColors];

        UFor( i, NColors )
            dscale[i] = sscale[i] = 0;

        scale_Point (&refldir, &normal, 2 * cos_normal);
        summ_Point (&refldir, dir, &refldir);

        Op_Point_2010( &isect ,+, origin ,mag*, dir );
        if (false && compute_bary_coords && 0 < scene->nvnmls)
        {
            real tmag;
                /* TODO: Will proper bump mapping ever happen?*/
            map_isect_height (&isect, &isect, &bpoint,
                              elem, scene->verts, scene->vnmls);
            if (hit_Plane (&tmag, &isect, &refldir, &simplex->plane))
                Op_Point_2010( &isect
                               ,+, &isect
                               ,   tmag*, &refldir );
        }

        UFor( i, space->nlights )
        {
            real tscale, magtolight;
            Point tolight;
            const PointLightSource* light;

            light = &space->lights[i];
            if (!light->on)  continue;
            diff_Point (&tolight, &light->location, &isect);

            magtolight = magnitude_Point (&tolight);
            scale_Point (&tolight, &tolight, 1 / magtolight);

            tscale = dot_Point (&tolight, &normal);
            if (tscale > 0)
            {
                real offset_magtolight;
                Point tmp_origin;

                offset_magtolight = magtolight - offset;
                Op_Point_2010( &tmp_origin ,-, &isect ,offset*, dir );

                if (cast_to_light (space, &tmp_origin, &tolight,
                                   offset_magtolight))
                {
                        /* real dist_factor *= 1 / (magtolight * magtolight); */

                        /* Add diffuse portion.*/
                    Op_2010( real, NColors, dscale
                             ,+, dscale
                             ,   tscale*, light->intensity );

                        /* Specular */
                    if (!light->diffuse && material)
                    {
                        real dot;
                        dot = dot_Point (&refldir, &tolight);
                        if (dot > 0)
                        {
                            tscale = pow (dot, material->shininess);
                            Op_10( real, NColors, sscale ,tscale+, sscale );
                        }
                    }
                }
            }
        }

        UFor( i, NColors )
        {
            real ambient, diffuse, specular;
            real tscale;

                /* If the cosine above (/diffuse_scale/) is 1,
                 * and the light intensity is 1 (assumed),
                 * and the material's diffuse portion is 1,
                 * then the resulting color value should be exactly 1.
                 * Thus, diffuse + ambient = 1
                 * before factoring in the cosine or material spec.
                 */
            ambient = image->ambient[i];
            diffuse = 1 - ambient;
            specular = 0;

            if (material)
            {
                ambient = material->ambient[i];
                diffuse = material->diffuse[i];
                specular = material->specular[i];
                if (specular > 0)  reflective = true;
            }

            tscale = ambient + diffuse * dscale[i] + specular * sscale[i];
            if (material && material->opacity < 1)
            {
                transparent = true;
                tscale *= material->opacity;
            }
            tscale = clamp_real (tscale, 0, 1);

            colors[i] *= tscale;
        }

        if (transparent)
        {
            Point tmp_origin, tmp_dir;
            real factors[NColors];
            UFor( i, NColors )
                factors[i] = ((1-material->opacity)
                              * material->transmission[i]);
            refraction_ray (&tmp_dir, dir, &normal,
                            material->optical_density, hit_front, cos_normal);

            Op_Point_2010( &tmp_origin ,+, &isect ,offset*, &tmp_dir );
            cast_colors (colors, space, image, &tmp_origin, &tmp_dir,
                         factors, nbounces);
        }
        if (reflective)
        {
            Point tmp_origin;
            real factors[NColors];
            UFor( i, NColors )
                factors[i] = (material->opacity * material->specular[i]);

            Op_Point_2010( &tmp_origin ,-, &isect ,offset*, dir );
            cast_colors (colors, space, image, &tmp_origin, &refldir,
                         factors, nbounces);
        }
    }

    UFor( i, NColors )  ret_colors[i] += colors[i];
}


static
    void
test_intersections (uint* ret_hit,
                    real* ret_mag,
                    const Ray* restrict ray,
                    uint nelemidcs,
                    __global const uint* restrict elemidcs,
                    __global const BarySimplex* restrict simplices,
                    __global const Simplex* restrict tris)
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

        tmp_hit = elemidcs[i];

        if (BarycentricRayTrace)
        {
            didhit = hit_BarySimplex (&tmp_mag, ray,
                                      &simplices[tmp_hit]);
        }
        else
        {
            didhit = hit_Simplex (&tmp_mag, *ray, tris[tmp_hit]);
        }

        if (didhit && tmp_mag < hit_mag)
        {
            hit_idx = tmp_hit;
            hit_mag = tmp_mag;
        }
    }


    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}


static
    void
cast_Ray (uint* restrict ret_hit, real* restrict ret_mag,
          const Ray* restrict ray,
          const uint nelems,
          __global const uint* restrict elemidcs,
          __global const KDTreeNode* restrict nodes,
          __global const BarySimplex* restrict simplices,
          __global const Simplex* restrict tris,
          __global const BoundingBox* restrict box,
          bool inside_box)
{
    Point invdirect;
    uint node_idx, parent = 0;
    uint hit_idx;
    real hit_mag;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;


    if (!KDTreeRayTrace)
    {
        test_intersections (&hit_idx, &hit_mag, ray,
                            nelems, elemidcs,
                            simplices, tris);
        *ret_hit = hit_idx;
        *ret_mag = hit_mag;
        return;
    }

    invmul_Point (&invdirect, &ray->direct);
    node_idx = first_KDTreeNode (&parent, ray,
                                 nodes, box, inside_box);

    while (node_idx != Max_uint)
    {
        __global const KDTreeLeaf* restrict leaf;

        leaf = &nodes[node_idx].as.leaf;
        test_intersections (&hit_idx, &hit_mag, ray,
                            leaf->nelems, &elemidcs[leaf->elemidcs],
                            simplices, tris);

        node_idx = next_KDTreeNode (&parent, ray, &invdirect,
                                    hit_mag,
                                    node_idx, nodes);
    }

    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
}

    void
cast1_ObjectRaySpace (uint* ret_hit, real* ret_mag,
                      const Point* origin,
                      const Point* direct,
                      const ObjectRaySpace* object,
                      bool inside_box)
{
    Ray ray;
    ray.origin = *origin;
    ray.direct = *direct;
    cast_Ray (ret_hit, ret_mag, &ray,
              object->nelems,
              object->tree.elemidcs, object->tree.nodes,
              object->simplices, object->elems,
              &object->box, inside_box);
}

    void
cast_nopartition (uint* ret_hit,
                  real* ret_mag,
                  uint* ret_object,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const Point* restrict dir,
                  bool inside_box,
                  uint ignore_object)
{
    uint i;
    uint hit_idx;
    real hit_mag;
    uint hit_object;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;
    hit_object = *ret_object;

    if (space->main.visible)
        cast1_ObjectRaySpace (&hit_idx, &hit_mag, origin, dir,
                              &space->main, inside_box);

    if (hit_idx < space->main.nelems)
        hit_object = space->nobjects;
        

    UFor( i, space->nobjects )
    {
        Point rel_origin, rel_dir;
        const ObjectRaySpace* object;
        bool rel_inside_box;
            /* Returns from ray cast.*/
        uint tmp_hit;
        real tmp_mag;

        if (i == ignore_object || !space->objects[i].visible)  continue;

        object = ray_to_ObjectRaySpace (&rel_origin, &rel_dir,
                                        origin, dir, space, i);

        rel_inside_box =
            inside_BoundingBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast1_ObjectRaySpace (&tmp_hit, &tmp_mag,
                              &rel_origin, &rel_dir,
                              object, rel_inside_box);

        if (tmp_mag < hit_mag)
        {
            hit_idx = tmp_hit;
            hit_mag = tmp_mag;
            hit_object = i;
        }
    }

    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
    *ret_object = hit_object;
}


static
    void
test_object_intersections (uint* ret_hit,
                           real* ret_mag,
                           uint* ret_object,
                           BitString* tested,
                           const Ray* ray,
                           uint nobjectidcs,
                           const uint* objectidcs,
                           const RaySpace* space)
{
    uint i;
    UFor( i, nobjectidcs )
    {
        uint objidx;
        Point rel_origin, rel_dir;
        const ObjectRaySpace* object;
        bool rel_inside_box;
            /* Returns from ray cast.*/
        uint tmp_hit;
        real tmp_mag;

        objidx = objectidcs[i];
        if (set1_BitString (tested, objidx))  continue;

        object = ray_to_ObjectRaySpace (&rel_origin, &rel_dir,
                                        &ray->origin, &ray->direct,
                                        space, objidx);

        rel_inside_box =
            inside_BoundingBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast1_ObjectRaySpace (&tmp_hit, &tmp_mag, &rel_origin, &rel_dir,
                              object, rel_inside_box);

        if (tmp_mag < *ret_mag)
        {
            *ret_hit = tmp_hit;
            *ret_mag = tmp_mag;
            *ret_object = objidx;
        }
    }
}


static
    void
cast_partitioned (uint* ret_hit,
                  real* ret_mag,
                  uint* ret_object,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const Point* restrict dir,
                  bool inside_box,
                  uint ignore_object)
{
    const uint ntestedbits = 128;
    Declare_BitString( tested, 128 );
    uint node_idx, parent = 0;
    const KDTreeNode* restrict nodes;
    const uint* restrict elemidcs;
    Point invdirect;
    uint hit_idx;
    real hit_mag;
    uint hit_object;
    Ray ray;

    copy_Point (&ray.origin, origin);
    copy_Point (&ray.direct, dir);

    assert (space->nobjects < ntestedbits);
    zero_BitString (tested, ntestedbits);
    if (ignore_object <= space->nobjects)
        set1_BitString (tested, ignore_object);

    nodes = space->object_tree.nodes;
    elemidcs = space->object_tree.elemidcs;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;
    hit_object = *ret_object;

    invmul_Point (&invdirect, &ray.direct);
    node_idx = first_KDTreeNode (&parent, &ray, nodes,
                                 &space->box, inside_box);

    while (node_idx != Max_uint)
    {
        __global const KDTreeLeaf* restrict leaf;

        leaf = &nodes[node_idx].as.leaf;
        test_object_intersections (&hit_idx, &hit_mag, &hit_object,
                                   tested,
                                   &ray, leaf->nelems,
                                   &elemidcs[leaf->elemidcs],
                                   space);
        node_idx = next_KDTreeNode (&parent, &ray,
                                    &invdirect,
                                    hit_mag,
                                    node_idx, nodes);
    }

    *ret_hit = hit_idx;
    *ret_mag = hit_mag;
    *ret_object = hit_object;
}

    void
cast_colors (real* ret_colors,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const Point* restrict origin,
             const Point* restrict dir,
             const real* factors,
             uint nbounces)
{
    uint i;
    real colors[NColors];
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_object = Max_uint;
    bool inside_box;

    if (nbounces >= image->nbounces_max)  return;

    if (space->partition)
    {
        inside_box = inside_BoundingBox (&space->box, origin);
        cast_partitioned (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);
    }
    else
    {
        inside_box = inside_BoundingBox (&space->main.box, origin);
        cast_nopartition (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);
    }

    UFor( i, NColors )  colors[i] = 0;
    fill_pixel (colors, hit_idx, hit_mag, hit_object,
                image, origin, dir, space, nbounces+1);
    UFor( i, NColors )  ret_colors[i] += factors[i] * colors[i];
}

    bool
cast_to_light (const RaySpace* restrict space,
               const Point* restrict origin,
               const Point* restrict dir,
               real magtolight)
{
    uint hit_idx = Max_uint;
    real hit_mag;
    uint hit_object = Max_uint;
    bool inside_box;
    hit_mag = magtolight;

    if (space->partition)
    {
        inside_box = inside_BoundingBox (&space->box, origin);
        cast_partitioned (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);
    }
    else
    {
        inside_box = inside_BoundingBox (&space->main.box, origin);
        cast_nopartition (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);
    }
    return hit_object > space->nobjects;
}


static
    void
cast_record (uint* hitline,
             real* magline,
             byte* pixline,
             uint col,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const Point* restrict origin,
             const Point* restrict dir,
             bool inside_box)
{
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_object = Max_uint;

    if (space->partition)
        cast_partitioned (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);
    else
        cast_nopartition (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          Max_uint);

    if (hitline)  hitline[col] = hit_idx;
    if (magline)  magline[col] = hit_mag;
    if (pixline)
    {
        uint i;
        real colors[NColors];
        UFor( i, NColors )  colors[i] = 0;
        fill_pixel (colors, hit_idx, hit_mag, hit_object,
                    image, origin, dir, space, 0);
        UFor( i, NColors )
        {
            pixline[3*col+i] = (byte)
                clamp_real (255.5 * colors[i], 0, 255.5);
        }
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
    const uint row_dim = UpDim;
    const uint col_dim = RightDim;
    const uint dir_dim = ForwardDim;
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

        if (image->hits)  hitline = &image->hits[row * image->stride];
        if (image->mags)  magline = &image->mags[row * image->stride];
        if (image->pixels)  pixline = &image->pixels[row * 3 * image->stride];

        row_angle = row_start + row_delta * row;

        UFor( col, ncols )
        {
            Point tdir, dir;
            real col_angle;
            col_angle = col_start + col_delta * col;

            zero_Point (&dir);
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
                         space, image,
                         origin, &dir, inside_box);
        }
    }
}


    void
rays_to_hits_fixed_plane (uint* hits, real* mags,
                          uint nrows, uint ncols, uint stride,
                          const RaySpace* space, real zpos)
{
    uint row;
    bool inside_box;
    const uint row_dim = UpDim;
    const uint col_dim = RightDim;
    const uint dir_dim = ForwardDim;
    Point origin;
    real col_start, row_start;
    real col_delta, row_delta;
    const BoundingBox* box;
    const ObjectRaySpace* object;

    object = &space->main;
    box = &object->box;

    row_start = box->min.coords[row_dim];
    row_delta = (box->max.coords[row_dim] - row_start) / nrows;
    row_start += row_delta / 2;

    col_start = box->min.coords[col_dim];
    col_delta = (box->max.coords[col_dim] - col_start) / ncols;
    col_start += col_delta / 2;

    inside_box = (zpos > box->min.coords[dir_dim] &&
                  zpos < box->max.coords[dir_dim]);

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

        hitline = &hits[row * stride];
        magline = &mags[row * stride];

        UFor( col, ncols )
        {
            Point dir;
            uint hit; real mag;

                /* if (! (row == 333 && col == 322))  continue; */

            zero_Point (&dir);
            dir.coords[row_dim] = row_start + row * row_delta;
            dir.coords[col_dim] = col_start + col * col_delta;

            diff_Point (&dir, &dir, &origin);
            normalize_Point (&dir, &dir);

            hit = object->nelems;
            mag = Max_real;
            cast1_ObjectRaySpace (&hit, &mag,
                                  &origin, &dir,
                                  object, inside_box);
            hitline[col] = hit;
            magline[col] = mag;

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}


    void
setup_ray_pixel_deltas_orthographic (Point* origin_start,
                                     Point* row_delta,
                                     Point* col_delta,
                                     uint nrows, uint ncols,
                                     const Point* origin,
                                     const PointXfrm* view_basis,
                                     real view_width)
{
    const uint row_dim = UpDim, col_dim = RightDim;
    uint max_n;
    Point diff;
    real tstart, tdelta;
    real eff_width;

    copy_Point (origin_start, origin);

    if (nrows >= ncols)  max_n = nrows;
    else                 max_n = ncols;

    tdelta = view_width / max_n;


    eff_width = (view_width * nrows) / max_n;
    tstart = (- eff_width + tdelta) / 2;

    scale_Point (row_delta, &view_basis->pts[row_dim], tdelta);
    scale_Point (&diff, &view_basis->pts[row_dim], tstart);
    summ_Point (origin_start, origin_start, &diff);

    eff_width = (view_width * ncols) / max_n;
    tstart = (- eff_width + tdelta) / 2;

    scale_Point (col_delta, &view_basis->pts[col_dim], tdelta);
    scale_Point (&diff, &view_basis->pts[col_dim], tstart);
    summ_Point (origin_start, origin_start, &diff);
}


    void
cast_row_orthographic (RayImage* restrict image,
                       uint row,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known)
{
    uint col, ncols;
    const BoundingBox* box;
    const Point* dir;
    Point partial_ray_origin;
    uint* hitline = 0;
    real* magline = 0;
    byte* pixline = 0;

    ncols = image->ncols;
    box = &space->box;

    if (image->hits)  hitline = &image->hits[row * image->stride];
    if (image->mags)  magline = &image->mags[row * image->stride];
    if (image->pixels)  pixline = &image->pixels[row * 3 * image->stride];

        /* For orthographic view, origin and direction storage are swapped.*/
    dir = &known->origin;
    scale_Point (&partial_ray_origin, &known->row_delta, row);
    summ_Point (&partial_ray_origin, &partial_ray_origin, &known->dir_start);

    UFor( col, ncols )
    {
        Point ray_origin;
        bool inside_box;

        scale_Point (&ray_origin, &known->col_delta, col);
        summ_Point (&ray_origin, &ray_origin, &partial_ray_origin);
        inside_box = inside_BoundingBox (box, &ray_origin);

        cast_record (hitline, magline, pixline, col,
                     space, image,
                     &ray_origin, dir,
                     inside_box);
    }
}


    void
setup_ray_pixel_deltas_perspective (Point* dir_start,
                                    Point* row_delta,
                                    Point* col_delta,
                                    uint nrows, uint ncols,
                                    const PointXfrm* view_basis,
                                    real view_angle)
{
    const uint row_dim = UpDim;
    const uint col_dim = RightDim;
    const uint dir_dim = ForwardDim;
    Point dstart, rdelta, cdelta;
    real halflen;
    halflen = sin (view_angle / 2);

    zero_Point (&dstart);
    zero_Point (&rdelta);
    zero_Point (&cdelta);

    if (nrows >= ncols)
    {
        dstart.coords[row_dim] = - halflen;
        dstart.coords[col_dim] = - (halflen * ncols) / nrows;
    }
    else
    {
        dstart.coords[row_dim] = - (halflen * nrows) / ncols;
        dstart.coords[col_dim] = - halflen;
    }
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
cast_row_perspective (RayImage* image, uint row,
                      const RaySpace* restrict space,
                      const RayCastAPriori* restrict known)
{
    uint col, ncols;
    const Point* origin;
    Point partial_dir;
    uint* hitline = 0;
    real* magline = 0;
    byte* pixline = 0;

    ncols = image->ncols;

    if (image->hits)  hitline = &image->hits[row * image->stride];
    if (image->mags)  magline = &image->mags[row * image->stride];
    if (image->pixels)  pixline = &image->pixels[row * 3 * image->stride];

    origin = &known->origin;
    Op_Point_2010( &partial_dir
                   ,+, &known->dir_start
                   ,   row*, &known->row_delta );

    UFor( col, ncols )
    {
        Point dir;

#if 0
        if (! (row == 10 && col == 10))
        {
            hitline[col] = Max_uint;
            continue;
        }
#endif

        Op_Point_2010( &dir
                       ,+, &partial_dir
                       ,   col*, &known->col_delta );
        normalize_Point (&dir, &dir);

        cast_record (hitline, magline, pixline, col,
                     space, image,
                     origin, &dir, known->inside_box);

#if 0
        {
            static bool missed = false;
            if (!missed && hitline[col] == Max_uint)
            {
                printf ("row:%u  col:%u\n", row, col);
                missed = true;
            }
        }
#endif
    }
}


#ifdef TrivialMpiRayTrace
    void
balancer_triv_sync_mpi_RayImage (RayImage* image,
                                 uint row_off, uint row_nul,
                                 uint nprocs)
{
    uint proc, ncols;
    ncols = image->ncols;
    for (proc = 1; proc < nprocs; ++proc)
    {
        uint i;
        for (i = proc; i < row_nul; i += nprocs)
        {
            MPI_Status status;
            uint row;
            row = row_off + i;
            if (image->hits)
                MPI_Recv (&image->hits[row*ncols],
                          ncols * sizeof(uint), MPI_BYTE,
                          proc, 1, MPI_COMM_WORLD, &status);
            if (image->mags)
                MPI_Recv (&image->mags[row*ncols],
                          ncols * sizeof(real), MPI_BYTE,
                          proc, 1, MPI_COMM_WORLD, &status);
            if (image->pixels)
                MPI_Recv (&image->pixels[row*3*ncols],
                          3 * ncols, MPI_BYTE,
                          proc, 1, MPI_COMM_WORLD, &status);
        }
    }
}

    void
computer_triv_sync_mpi_RayImage (const RayImage* image,
                                 uint row_off, uint row_nul,
                                 uint myrank, uint nprocs)
{
    uint i, ncols;
    ncols = image->ncols;
    for (i = myrank; i < row_nul; i += nprocs)
    {
        uint row;
        row = row_off + i;
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

    void
setup_RayCastAPriori (RayCastAPriori* dst,
                      const RayImage* image,
                      const Point* origin,
                      const PointXfrm* view_basis,
                      const BoundingBox* box)
{
    if (image->perspective)
    {
        copy_Point (&dst->origin, origin);
        setup_ray_pixel_deltas_perspective (&dst->dir_start,
                                            &dst->row_delta,
                                            &dst->col_delta,
                                            image->nrows, image->ncols,
                                            view_basis,
                                            image->hifov);
        dst->inside_box = inside_BoundingBox (box, origin);
    }
    else
    {
        copy_Point (&dst->origin, &view_basis->pts[ForwardDim]);
        setup_ray_pixel_deltas_orthographic (&dst->dir_start,
                                             &dst->row_delta,
                                             &dst->col_delta,
                                             image->nrows, image->ncols,
                                             origin, view_basis,
                                             image->hifov);
        dst->inside_box = false;
    }
}


    /* Expect /origin/ and /dir/ to already
     * represent the middle line of sight.
     */
    void
ray_from_RayCastAPriori (Point* origin, Point* dir,
                         const RayCastAPriori* known,
                         uint row, uint col,
                         const RayImage* image)
{
    if (!image->perspective)
    {
        copy_Point (dir, &known->origin);
        Op_Point_2021010( origin
                          ,+, &known->dir_start
                          ,   +, row*, &known->row_delta
                          ,      col*, &known->col_delta );
    }
    else
    {
        copy_Point (origin, &known->origin);

        Op_Point_2021010( dir
                          ,+, &known->dir_start
                          ,   +, row*, &known->row_delta
                          ,      col*, &known->col_delta );
        normalize_Point (dir, dir);
    }
}

    void
cast_partial_RayImage (RayImage* restrict image,
                       uint row_off,
                       uint row_nul,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known)
{
    uint i, inc, nprocs, myrank;

#ifdef TrivialMpiRayTrace
    MPI_Comm_size (MPI_COMM_WORLD, (int*) &nprocs);
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &myrank);
#else
    myrank = 0;
    nprocs = 1;
#endif

#ifdef PackOpsAvail
    inc = RayPacketDimSz * nprocs;
#else
    inc = nprocs;
#endif

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (i = myrank; i < row_nul; i += inc)
    {
#ifdef PackOpsAvail
        uint j;
        if (RayPacketDimSz <= row_nul)
        {
            for (j = 0; j < image->ncols; j += RayPacketDimSz)
                cast_packet_RayImage (image, row_off + i, j, space, known);
        }
        else
        {
            uint n;
            n = row_nul - i;
            UFor( j, n )
                if (image->perspective)
                    cast_row_perspective (image, row_off + i + j, space, known);
                else
                    cast_row_orthographic (image, row_off + i + j, space, known);
        }
#else
        if (image->perspective)
            cast_row_perspective (image, row_off + i, space, known);
        else
            cast_row_orthographic (image, row_off + i, space, known);
#endif
    }

#ifdef TrivialMpiRayTrace
    if (myrank == 0)
        balancer_triv_sync_mpi_RayImage (image, row_off, row_nul, nprocs);
    else
        computer_triv_sync_mpi_RayImage (image, row_off, row_nul,
                                         myrank, nprocs);
#endif
}


    void
cast_RayImage (RayImage* restrict image,
               const RaySpace* restrict space,
               const Point* restrict origin,
               const PointXfrm* restrict view_basis)
{
    RayCastAPriori known;
    setup_RayCastAPriori (&known, image, origin, view_basis,
                          &space->box);
    cast_partial_RayImage (image, 0, image->nrows,
                           space, &known);
}

#endif  /* #ifndef __OPENCL_VERSION__ */

