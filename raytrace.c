
#ifndef __OPENCL_VERSION__
#include "raytrace.h"

#include "affine.h"
#include "bbox.h"
#include "cx/bittable.h"
#include "color.h"
#include "lightcut.h"
#include "order.h"
#include "point.h"
#include "simplex.h"
#include "space-junk.h"
#include "xfrm.h"

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
init_Scene_KPTreeGrid (KPTreeGrid* grid, const Scene* scene);
static void
init_RaySpace_KDTreeGrid (KDTreeGrid* grid, const RaySpace* space);

static void
fill_pixel (Color* ret_color,
            uint hitidx, real mag, uint objidx,
            const RayImage* image,
            const Point* origin,
            const Point* dir,
            const RaySpace* space,
            Trit front,
            uint nbounces,
            GMRand* gmrand);
static void
cast_colors (Color* ret_color,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const Point* restrict origin,
             const Point* restrict dir,
             const Color* factor,
             Trit front,
             uint nbounces,
             GMRand* gmrand);
static void
cast_row_orthographic (RayImage* restrict image,
                       uint row,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known);
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
    init_LightCutTree (&space->lightcuts);
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
    set_Color (&light->intensity, 1);
    light->diffuse = false;
    light->hemisphere = false;
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
    init_BBox (&space->box, scene->nverts, scene->verts);

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
    object->tree.nodes[0].as.leaf.box = object->box;
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
    lose_KDTree (&space->object_tree);
    lose_LightCutTree (&space->lightcuts);
}

    void
cleanup_ObjectRaySpace (ObjectRaySpace* space)
{
    cleanup_Scene (&space->scene);
    lose_KDTree (&space->tree);
    lose_KPTree (&space->verttree);
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
        space->box = grid.box;
            /* Since it's a regeneration, clean up the previous version.*/
        lose_KDTree (&space->object_tree);
        build_KDTree (&space->object_tree, &grid, 0);
            /* output_KDTreeGrid (stderr, &grid); */
            /* output_KDTree (stderr, &space->object_tree); */
        lose_KDTreeGrid (&grid);
    }
    else
    {
        space->partition = false;
        space->box = space->main.box;
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
    printf ("nnodes:%u  nelemidcs:%u\n",
            space->tree.nnodes, space->tree.nelemidcs);
#endif
    lose_KDTreeGrid (&grid);
}

    void
partition_verts_ObjectRaySpace (ObjectRaySpace* space)
{
    KPTreeGrid grid;
    init_Scene_KPTreeGrid (&grid, &space->scene);
    build_KPTree (&space->verttree, &grid);
    lose_KPTreeGrid (&grid);
}

    void
init_Scene_KDTreeGrid (KDTreeGrid* grid, const Scene* scene,
                       const BBox* box)
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
                assert (inside_BBox (box, p));
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
    grid->box = *box;
}

    void
init_Scene_KPTreeGrid (KPTreeGrid* grid, const Scene* scene)
{
    init_KPTreeGrid (grid, scene->nverts);

    {:for (i ; scene->nverts)
        set1_KPTreeGrid (grid, i, &scene->verts[i]);
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
        BBox box;

        if (i < space->nobjects)  object = &space->objects[i];
        else                      object = &space->main;

        if (!object->visible)  continue;

        ti = 2 * nvisible;

        if (i < space->nobjects)
            trxfrm_BBox (&box,
                         &object->orientation,
                         &object->box,
                         &object->centroid);
        else
            box = object->box;

        include_BBox (&grid->box, &grid->box, &box);

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
        /* image->hifov = 60 * M_PI / 180; */
    image->hifov = 2 * atan (1.0 / 3);
    image->perspective = true;
    UFor( i, NColors )
        image->ambient[i] = 0.2;
    image->view_light = 0;
    image->shading_on = true;
    image->color_distance_on = true;
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
        follow_Point (normal, normal, &vnmls[elem->vnmls[i]],
                      bpoint->coords[i]);
    }
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
    Claim2( objidx ,<=, space->nobjects );
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
        *ret_origin = *origin;
        *ret_dir = *dir;
    }
    return object;
}

    /** Assume no solids inside one another.
     * Otherwise, we'd need to track an IOR.
     **/
static
    void
refraction_ray (Point* dst, const Point* dir, const Point* normal,
                real r, bool entering, real cos_normal)
{
    Point a, b;
    real d;
    if (r == 1)
    {
        *dst = *dir;
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
    normalize_Point (dst, dst);
}

static
    uint
splitting_plane_count (const Point* origin, const Point* direct, real mag,
                       const KDTree* tree, const BBox* box)
{
    uint count = 0;
    uint node_idx, parent, destin_nodeidx;
    Point destin;
    Point invdirect;
    Ray ray;
    bool inside_box;

    ray.origin = *origin;
    ray.direct = *direct;

    follow_Ray (&destin, &ray, mag);
    if (inside_BBox (box, &destin))
        destin_nodeidx = find_KDTreeNode (&parent, &destin,
                                          tree->nodes);
    else
        destin_nodeidx = Max_uint;

#if 0
        /* Return the number of elements in the hit node.*/
    return ((destin_nodeidx == Max_uint) ? 0 :
            tree->nodes[destin_nodeidx].as.leaf.nelems);
#endif

    inside_box = inside_BBox (box, origin);

    reci_Point (&invdirect, direct);
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

    /** Get the ambient, diffuse, and specular components
     * of a pixel without considering illumination.
     **/
static void
pixel_from_Material (Color* ambient, Color* diffuse,
                     Color* specular, Color* emissive,
                     const Material* matl,
                     const BaryPoint* texpoint,
                     const Scene* scene)
{
    if (!matl)
    {
            /* If the cosine between light and normal is 1,
             * and the light intensity is 1,
             * then the resulting color value should be exactly 1.
             * Thus, diffuse + ambient = 1 by default.
             */
        set_Color (ambient, .2);
        set_Color (diffuse, .8);
        set_Color (specular, 0);
        return;
    }

    *ambient = matl->ambient;
    *diffuse = matl->diffuse;
    *specular = matl->specular;
    *emissive = matl->emissive;

        /* Texture mapping.*/
    if (texpoint)
    {
        const Texture* tex;
        Color color;
        real alpha;
        if (matl->ambient_texture != Max_uint)
        {
            tex = &scene->txtrs[matl->ambient_texture];
            alpha = map_Texture (&color, tex, texpoint);
            mix_Color (ambient, ambient, &color, alpha);
        }
        if (matl->diffuse_texture != Max_uint)
        {
            tex = &scene->txtrs[matl->diffuse_texture];
            alpha = map_Texture (&color, tex, texpoint);
            mix_Color (diffuse, diffuse, &color, alpha);
        }
        if (matl->specular_texture != Max_uint)
        {
            tex = &scene->txtrs[matl->specular_texture];
            alpha = map_Texture (&color, tex, texpoint);
            mix_Color (specular, specular, &color, alpha);
        }
    }
}


    void
fill_pixel (Color* ret_color,
            uint hitidx, real mag, uint objidx,
            const RayImage* image,
            const Point* origin,
            const Point* dir,
            const RaySpace* space,
            Trit front,
            uint nbounces,
            GMRand* gmrand)
{
    const bool shade_by_element = false;
    const bool color_by_element = false;
    const bool compute_bary_coords = true;
    const bool show_splitting_planes = false;
    bool miss_effects = false;
    Color color;
    const BarySimplex* simplex;
    const ObjectRaySpace* object;
    Point rel_origin, rel_dir;
    const Scene* scene;
    const SceneElement* elem;
    const Material* material = 0;
    bool hit_front;
    real cos_normal;
    Point bpoint, normal;
    BaryPoint texpoint;
    uint i;

    if (show_splitting_planes)
        miss_effects = true;

    if (objidx > space->nobjects)
    {
        if (space->skytxtr < space->main.scene.ntxtrs)
        {
            map_sky_Texture (ret_color,
                             &space->main.scene.txtrs[space->skytxtr],
                             dir);
        }
        else
        {
            zero_Color (ret_color);
        }

        if (!miss_effects)  return;
        color = *ret_color;
    }
    else
    {
        set_Color (&color, 1);
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
                color.coords[i] = clamp_real (1 - red * (1 - color.coords[i]), 0, 1);
            else
                color.coords[i] = clamp_real (red * color.coords[i], 0, 1);
        }
    }

    if (objidx > space->nobjects)
    {
        summ_Color (ret_color, ret_color, &color);
        return;
    }

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
        zero_Color (&color);
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
                    color.coords[idx1] = 1;
                    color.coords[idx2] = val - i;
                }
                else
                {
                    color.coords[idx1] = i+1 - val;
                    color.coords[idx2] = 1;
                }
                break;
            }
        }
    }

    if (compute_bary_coords)
    {
        Point rel_isect;
        follow_Point (&rel_isect, &rel_origin, &rel_dir, mag);
        barycentric_Point (&bpoint, &rel_isect, simplex);

        if (elem->txpts[0] < Max_uint)
        {
            Op_s( real, NDimensions-1, texpoint.coords , 0 );

            UFor( i, NDimensions )
            {
                assert (elem->txpts[i] < Max_uint);
                Op_2020s( real, NDimensions-1, texpoint.coords
                          ,+, texpoint.coords
                          ,   *, scene->txpts[elem->txpts[i]].coords
                          ,      bpoint.coords[i] );
            }
        }
    }


        /* Get the normal.*/
    if (material && material->bump_texture < Max_uint)
        map_bump_Texture (&normal, &scene->txtrs[material->bump_texture],
                          &texpoint);
    else if (compute_bary_coords && 0 < scene->nvnmls)
        map_vertex_normal (&normal, scene->vnmls, elem, &bpoint);
    else
        copy_Point (&normal, &simplex->plane.normal);
        /* Rotate it when necessary.*/
    if (objidx != space->nobjects)
        trxfrm_Point (&normal, &object->orientation, &normal);
        /* The above cases do not give a normalized vector!*/
    normalize_Point (&normal, &normal);

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
        color.coords[0] *= (real) ((y & 0xFF0000) >> 16) / (nincs-1);
        color.coords[1] *= (real) ((y & 0x00FF00) >>  8) / (nincs-1);
        color.coords[2] *= (real) ((y & 0x0000FF) >>  0) / (nincs-1);
    }

    if (shade_by_element)
    {
        real scale = 0;
        scale = 1 - (real) hitidx / scene->nelems;
        scale_Color (&color, &color, scale);
    }
    else if (image->shading_on)
    {
        Point isect, refldir;
        Color dscale, sscale;
        Color ambient, diffuse, specular, emissive;
        pixel_from_Material (&ambient, &diffuse,
                             &specular, &emissive,
                             material,
                             (compute_bary_coords ? &texpoint : 0),
                             scene);

        zero_Color (&dscale);
        zero_Color (&sscale);

        follow_Point (&refldir, dir, &normal, 2 * cos_normal);

        follow_Point (&isect, origin, dir, mag);

        {:for (light_idx ; space->nlights)
            real tscale, magtolight;
            Point tolight;
            const PointLightSource* const light = &space->lights[light_idx];

            if (!light->on)  continue;
            diff_Point (&tolight, &light->location, &isect);

            magtolight = magnitude_Point (&tolight);
            scale_Point (&tolight, &tolight, 1 / magtolight);

            tscale = dot_Point (&tolight, &normal);
            if (tscale > 0)
            {
                Ray tolight_ray;
                if (light->hemisphere)
                {
                    real dot = - dot_Point (&tolight, &light->direct);
                    if (dot <= 0)  continue;
                    tscale *= dot;
                }
                tolight_ray.origin = isect;
                tolight_ray.direct = tolight;
                if (cast_to_light (space, &tolight_ray,
                                   front,
                                   magtolight))
                {
                    real dist_factor = 1;
                    tscale *= dist_factor;

                        /* Add diffuse portion.*/
                    follow_Color (&dscale, &dscale,
                                  &light->intensity, tscale);

                        /* Specular */
                    if (!light->diffuse && material)
                    {
                        real dot;
                        dot = dot_Point (&refldir, &tolight);
                        if (dot > 0)
                        {
                            tscale = pow (dot, material->shininess);
                            Op_20s( real, NColors, sscale.coords
                                   ,+, sscale.coords , tscale );
                        }
                    }
                }
            }
        }

        UFor( i, NColors )
            color.coords[i] *=
                ambient.coords[i]
                + diffuse.coords[i] * dscale.coords[i]
                + specular.coords[i] * sscale.coords[i]
                + emissive.coords[i];

        if (space->lightcuts.nodes.sz > 0)
        {
            Color tmp;
            RayHit hit;
            hit.isect = isect;
            negate_Point (&hit.incid, dir);
            hit.normal = normal;
            hit.front = front;
            hit.mag = mag;
            cast_LightCutTree (&tmp, &space->lightcuts,
                               &diffuse, &hit, space, gmrand);
            summ_Color (&color, &color, &tmp);
        }

        if (material && material->opacity < 1)
        {
            Point tmp_dir;
            Color factor;

            scale_Color (&color, &color, material->opacity);
            scale_Color (&factor, &material->transmission,
                         1 - material->opacity);
            refraction_ray (&tmp_dir, dir, &normal,
                            material->optical_density, hit_front, cos_normal);

            cast_colors (&color, space, image, &isect, &tmp_dir,
                         &factor, front == Nil ? Yes : Nil, nbounces, gmrand);
        }
        if (material && material->reflective)
        {
            Color factor;
            scale_Color (&factor, &material->specular, material->opacity);

            cast_colors (&color, space, image, &isect, &refldir,
                         &factor, front, nbounces, gmrand);
        }
    }

    summ_Color (ret_color, ret_color, &color);
}


static
    void
test_intersections (uint* ret_hit,
                    real* ret_mag,
                    const Ray* restrict ray,
                    uint nelemidcs,
                    __global const uint* restrict elemidcs,
                    __global const BarySimplex* restrict simplices,
                    __global const Simplex* restrict tris,
                    Trit front)
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
                                      &simplices[tmp_hit],
                                      front);
        }
        else
        {
            didhit = hit_Simplex (&tmp_mag, *ray, tris[tmp_hit],
                                  front);
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
          __global const BBox* restrict box,
          bool inside_box,
          Trit front)
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
                            simplices, tris, front);
        *ret_hit = hit_idx;
        *ret_mag = hit_mag;
        return;
    }

    reci_Point (&invdirect, &ray->direct);
    node_idx = first_KDTreeNode (&parent, ray,
                                 nodes, box, inside_box);

    while (node_idx != Max_uint)
    {
        __global const KDTreeLeaf* restrict leaf;

        leaf = &nodes[node_idx].as.leaf;
        test_intersections (&hit_idx, &hit_mag, ray,
                            leaf->nelems, &elemidcs[leaf->elemidcs],
                            simplices, tris, front);

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
                      bool inside_box,
                      Trit front)
{
    Ray ray;
    ray.origin = *origin;
    ray.direct = *direct;
    cast_Ray (ret_hit, ret_mag, &ray,
              object->nelems,
              object->tree.elemidcs, object->tree.nodes,
              object->simplices, object->elems,
              &object->box, inside_box, front);
}

    void
cast_nopartition (uint* ret_hit,
                  real* ret_mag,
                  uint* ret_object,
                  const RaySpace* restrict space,
                  const Point* restrict origin,
                  const Point* restrict dir,
                  bool inside_box,
                  Trit front,
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
                              &space->main, inside_box, front);

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
            inside_BBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast1_ObjectRaySpace (&tmp_hit, &tmp_mag,
                              &rel_origin, &rel_dir,
                              object, rel_inside_box, front);

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
                           BitTable tested,
                           const Ray* ray,
                           uint nobjectidcs,
                           const uint* objectidcs,
                           const RaySpace* space,
                           Trit front)
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
        if (set1_BitTable (tested, objidx))  continue;

        object = ray_to_ObjectRaySpace (&rel_origin, &rel_dir,
                                        &ray->origin, &ray->direct,
                                        space, objidx);

        rel_inside_box =
            inside_BBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast1_ObjectRaySpace (&tmp_hit, &tmp_mag, &rel_origin, &rel_dir,
                              object, rel_inside_box, front);

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
                  Trit front,
                  uint ignore_object)
{
    const uint ntestedbits = 128;
    FixDeclBitTable( tested, 128, 0 );
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
    if (ignore_object <= space->nobjects)
      set1_BitTable (tested, ignore_object);

    nodes = space->object_tree.nodes;
    elemidcs = space->object_tree.elemidcs;

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;
    hit_object = *ret_object;

    reci_Point (&invdirect, &ray.direct);
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
                                   space, front);
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
cast1_RaySpace (uint* ret_hit, real* ret_mag,
                uint* ret_objidx,
                const Ray* ray,
                const RaySpace* space,
                Trit front)
{
    bool inside_box = inside_BBox (&space->box, &ray->origin);
    if (space->partition)
        cast_partitioned (ret_hit,
                          ret_mag,
                          ret_objidx,
                          space,
                          &ray->origin,
                          &ray->direct,
                          inside_box,
                          front,
                          Max_uint);
    else
        cast_nopartition (ret_hit, ret_mag, ret_objidx,
                          space,
                          &ray->origin,
                          &ray->direct,
                          inside_box,
                          front,
                          Max_uint);
}

    void
cast_colors (Color* ret_color,
             const RaySpace* restrict space,
             const RayImage* restrict image,
             const Point* restrict origin,
             const Point* restrict dir,
             const Color* factor,
             Trit front,
             uint nbounces,
             GMRand* gmrand)
{
    Color color;
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_object = Max_uint;
    bool inside_box;

    if (nbounces >= image->nbounces_max)  return;

    if (space->partition)
    {
        inside_box = inside_BBox (&space->box, origin);
        cast_partitioned (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          front,
                          Max_uint);
    }
    else
    {
        inside_box = inside_BBox (&space->main.box, origin);
        cast_nopartition (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          front,
                          Max_uint);
    }

    zero_Color (&color);
    fill_pixel (&color, hit_idx, hit_mag, hit_object,
                image, origin, dir, space, front, nbounces+1,
                gmrand);
    prod_Color (&color, &color, factor);
    summ_Color (ret_color, ret_color, &color);
}

    bool
cast_to_light (const RaySpace* restrict space,
               const Ray* restrict ray,
               Trit front,
               real magtolight)
{
    uint hit_idx = Max_uint;
    real hit_mag = magtolight;
    uint hit_object = Max_uint;

    cast1_RaySpace (&hit_idx, &hit_mag, &hit_object, ray, space, front);
    return approx_eql (magtolight, hit_mag, 1, 1e2);
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
             bool inside_box,
             GMRand* gmrand)
{
    uint hit_idx = Max_uint;
    real hit_mag = Max_real;
    uint hit_object = Max_uint;
    const Trit front = Yes;

    if (space->partition)
        cast_partitioned (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          front,
                          Max_uint);
    else
        cast_nopartition (&hit_idx, &hit_mag, &hit_object,
                          space, origin, dir, inside_box,
                          front,
                          Max_uint);

    if (hitline)  hitline[col] = hit_idx;
    if (magline)  magline[col] = hit_mag;
    if (pixline)
    {
        Color color;
        zero_Color (&color);
        fill_pixel (&color, hit_idx, hit_mag, hit_object,
                    image, origin, dir, space, front, 0,
                    gmrand);
        {:for (i ; NColors)
            pixline[3*col+i] = (byte)
                clamp_real (255.5 * color.coords[i], 0, 255.5);
        }
    }
}


#ifndef __OPENCL_VERSION__
    void
rays_to_hits_fish (RayImage* restrict image,
                   const RaySpace* space,
                   const Point* origin,
                   const PointXfrm* view_basis,
                   real view_angle)
{
#ifndef _WIN32
    uint row;
#else
    int row;
#endif
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

    inside_box = inside_BBox (&space->box, origin);

#pragma omp parallel for schedule(dynamic)
    UFor( row, nrows )
    {
        uint col;
        real row_angle;
        uint* hitline = 0;
        real* magline = 0;
        byte* pixline = 0;
        GMRand gmrand;

        // Setup is poor, arbitrary numbers.
        init2_GMRand (&gmrand, row % 2, 2);
        step_GMRand (&gmrand, (row * 2) % 17);

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
                         origin, &dir, inside_box,
                         &gmrand);
        }
    }
}


    void
rays_to_hits_fixed_plane (uint* hits, real* mags,
                          uint nrows, uint ncols, uint stride,
                          const RaySpace* space, real zpos)
{
#ifndef _WIN32
    uint row;
#else
    int row;
#endif
    bool inside_box;
    const uint row_dim = UpDim;
    const uint col_dim = RightDim;
    const uint dir_dim = ForwardDim;
    Point origin;
    real col_start, row_start;
    real col_delta, row_delta;
    const BBox* box;
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

    inside_box = inside_BBox (box, &origin);

#pragma omp parallel for schedule(dynamic)
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
                                  object, inside_box, Yes);
            hitline[col] = hit;
            magline[col] = mag;

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}


    void
cast_row_orthographic (RayImage* restrict image,
                       uint row,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known)
{
    uint col, ncols;
    const BBox* box;
    const Point* dir;
    uint* hitline = 0;
    real* magline = 0;
    byte* pixline = 0;
    GMRand gmrand;

    // Setup is poor, arbitrary numbers.
    init2_GMRand (&gmrand, row % 2, 2);
    step_GMRand (&gmrand, (row * 2) % 17);

    ncols = image->ncols;
    box = &space->box;

    if (image->hits)  hitline = &image->hits[row * image->stride];
    if (image->mags)  magline = &image->mags[row * image->stride];
    if (image->pixels)  pixline = &image->pixels[row * 3 * image->stride];

    dir = &known->basis.pts[FwDim];

    UFor( col, ncols )
    {
        Point ray_origin;
        bool inside_box;

        ray_origin = known->origin;
        follow_Point (&ray_origin, &ray_origin,
                      &known->basis.pts[UpDim],
                      known->up_scale * (-1 + (2*row+1.0) / image->nrows));
        follow_Point (&ray_origin, &ray_origin,
                      &known->basis.pts[RtDim],
                      known->rt_scale * (-1 + (2*col+1.0) / image->ncols));

        inside_box = inside_BBox (box, &ray_origin);

        cast_record (hitline, magline, pixline, col,
                     space, image,
                     &ray_origin, dir,
                     inside_box, &gmrand);
    }
}


    void
cast_row_perspective (RayImage* image, uint row,
                      const RaySpace* restrict space,
                      const RayCastAPriori* restrict known)
{
    uint col, ncols;
    const Point* origin;
    uint* hitline = 0;
    real* magline = 0;
    byte* pixline = 0;
    GMRand gmrand;

    // Setup is poor, arbitrary numbers.
    init2_GMRand (&gmrand, row % 2, 2);
    step_GMRand (&gmrand, (row * 2) % 17);

    ncols = image->ncols;

    if (image->hits)  hitline = &image->hits[row * image->stride];
    if (image->mags)  magline = &image->mags[row * image->stride];
    if (image->pixels)  pixline = &image->pixels[row * 3 * image->stride];

    origin = &known->origin;

    UFor( col, ncols )
    {
        Point dir;

#if 0
        if (! (image->nrows - 1 - row == 31 && col == 27) &&
            ! (image->nrows - 1 - row == 31 && col == 28) &&
            ! (image->nrows - 1 - row == 31 && col == 26))
        {
            if (hitline)  hitline[col] = Max_uint;
            if (magline)  magline[col] = Max_real;
            if (pixline)
            {
                pixline[3*col+0] = 0;
                pixline[3*col+1] = 0;
                pixline[3*col+2] = 0;
            }
            continue;
        }
        fprintf (stderr, "\ncol:%u  ", col);
#endif

        zero_Point (&dir);
        dir.coords[UpDim] = known->up_scale * (-1 + (2*row+1.0) / image->nrows);
        dir.coords[RtDim] = known->rt_scale * (-1 + (2*col+1.0) / image->ncols);
        dir.coords[FwDim] = 1;
        trxfrm_Point (&dir, &known->basis, &dir);
        normalize_Point (&dir, &dir);

        cast_record (hitline, magline, pixline, col,
                     space, image,
                     origin, &dir, known->inside_box, &gmrand);

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
                      const BBox* box)
{
    dst->origin = *origin;
    dst->basis = *view_basis;
    if (image->perspective)
    {
        dst->up_scale = tan(.5 * image->hifov);
        dst->inside_box = inside_BBox (box, origin);
    }
    else
    {
        dst->up_scale = .5 * image->hifov;
        dst->inside_box = false;
    }
    dst->rt_scale = dst->up_scale * ((real)image->ncols / image->nrows);
}


    void
ray_from_RayCastAPriori (Ray* ray,
                         const RayCastAPriori* known,
                         uint row, uint col,
                         const RayImage* image)
{
    ray->origin = known->origin;
    if (image->perspective)
    {
        zero_Point (&ray->direct);
        ray->direct.coords[UpDim] =
            known->up_scale * (-1 + (2*row+1.0) / image->nrows);
        ray->direct.coords[RtDim] =
            known->rt_scale * (-1 + (2*col+1.0) / image->ncols);
        ray->direct.coords[FwDim] = 1;
        trxfrm_Point (&ray->direct, &known->basis, &ray->direct);
        normalize_Point (&ray->direct, &ray->direct);
    }
    else
    {
        follow_Point (&ray->origin, &ray->origin,
                      &known->basis.pts[UpDim],
                      known->up_scale * (-1 + (2*row+1.0) / image->nrows));
        follow_Point (&ray->origin, &ray->origin,
                      &known->basis.pts[RtDim],
                      known->rt_scale * (-1 + (2*col+1.0) / image->ncols));

        ray->direct = known->basis.pts[FwDim];
    }
}

    void
cast_partial_RayImage (RayImage* restrict image,
                       uint row_off,
                       uint row_nul,
                       const RaySpace* restrict space,
                       const RayCastAPriori* restrict known)
{
#ifndef _WIN32
    uint i;
#else
    int i;
#endif
    uint inc, nprocs, myrank;

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

#pragma omp parallel for schedule(dynamic)
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

