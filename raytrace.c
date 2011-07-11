
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
setup_ray_pixel_deltas_orthographic (Point* dir_start,
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
}

    void
init_PointLightSource (PointLightSource* light)
{
    zero_Point (&light->location);
    light->intensity = 1;
}

    void
init_filled_RaySpace (RaySpace* space)
{
    uint i;
    init_filled_ObjectRaySpace (&space->main);
    UFor( i, space->nobjects )
        init_filled_ObjectRaySpace (&space->objects[i]);
}

    void
init_filled_ObjectRaySpace (ObjectRaySpace* space)
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
        if (!good)  printf ("ei:%u\n", ei);
        assert (good);
    }

    partition_ObjectRaySpace (space);
    partition_verts_ObjectRaySpace (space);
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
        build_KDTree (&space->object_tree, &grid);
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
    build_KDTree (&space->tree, &grid);
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
    grid->nintls = 2 * (1 + space->nobjects);
    grid->intls[0] = AllocT( uint, NDimensions * grid->nintls );
    grid->coords[0] = AllocT( real, NDimensions * grid->nintls );

    UFor( i, NDimensions )
    {
        uint ti;
        ti = 2 * space->nobjects;

        grid->intls[i] = &grid->intls[0][i * grid->nintls];
        grid->coords[i] = &grid->coords[0][i * grid->nintls];

            /* Main case (special).*/
        grid->intls[i][ti] = ti;
        grid->intls[i][ti+1] = ti+1;
        grid->coords[i][ti] = space->main.box.min.coords[i];
        grid->coords[i][ti+1] = space->main.box.max.coords[i];
        grid->box.min.coords[i] = space->main.box.min.coords[i];
        grid->box.max.coords[i] = space->main.box.max.coords[i];
    }

    UFor( i, space->nobjects )
    {
        uint dim, ti;
        const ObjectRaySpace* object;
        BoundingBox box;

        ti = 2 * i;
        object = &space->objects[i];
        
        trxfrm_BoundingBox (&box,
                            &object->orientation,
                            &object->box,
                            &object->centroid);

        UFor( dim, NDimensions )
        {
            real lo, hi;
            lo = box.min.coords[dim];
            hi = box.max.coords[dim];
            grid->intls[dim][ti] = ti;
            grid->intls[dim][ti+1] = ti+1;
            grid->coords[dim][ti] = lo;
            grid->coords[dim][ti+1] = hi;
            if (lo < grid->box.min.coords[dim])
                grid->box.min.coords[dim] = lo;
            if (hi > grid->box.max.coords[dim])
                grid->box.max.coords[dim] = hi;
        }
    }
}


void init_RayImage (RayImage* image)
{
    uint i;
    image->hits = 0;
    image->mags = 0;
    image->pixels = 0;
    image->nrows = 0;
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


static
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
        Point tmp;
        assert (elem->vnmls[i] != Max_uint);
        scale_Point (&tmp, &vnmls[elem->vnmls[i]], bpoint->coords[i]);
        summ_Point (normal, normal, &tmp);
    }
    normalize_Point (normal, normal);
}

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

    UFor( i, NColors )  colors[i] = 1;

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
        scale_Point (&rel_isect, &rel_dir, mag);
        summ_Point (&rel_isect, &rel_isect, &rel_origin);
        bpoint.coords[0] = 1;
        UFor( i, NDimensions-1 )
        {
            bpoint.coords[i+1] =
                distance_Plane (&simplex->barys[i], &rel_isect);
            bpoint.coords[0] -= bpoint.coords[i+1];
        }
    }

        /* Get the normal.*/
    if (compute_bary_coords && 0 < scene->nvnmls)
        map_vertex_normal (&normal, scene->vnmls, elem, &bpoint);
    else
        copy_Point (&normal, &simplex->plane.normal);
        /* Rotate it when necessary.*/
    if (objidx != space->nobjects)
    {
        Point tmp;
        trxfrm_Point (&tmp, &object->orientation, &normal);
        copy_Point (&normal, &tmp);
    }

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
    else if (material && material->ambient_texture != Max_uint)
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
        real dscale = 0, sscale = 0;

        scale_Point (&isect, dir, mag);
        summ_Point (&isect, &isect, origin);

        scale_Point (&refldir, &normal, 2 * cos_normal);
        summ_Point (&refldir, dir, &refldir);

        UFor( i, space->nlights )
        {
            real tscale, magtolight;
            Point tolight;
            const PointLightSource* light;

            light = &space->lights[i];
            diff_Point (&tolight, &light->location, &isect);

            magtolight = magnitude_Point (&tolight);
            scale_Point (&tolight, &tolight, 1 / magtolight);

            tscale = dot_Point (&tolight, &normal);
            if (tscale > 0)
            {
                real offset_magtolight;
                Point tmp_origin;

                offset_magtolight = magtolight - offset;
                scale_Point (&tmp_origin, &tolight, offset);
                summ_Point (&tmp_origin, &tmp_origin, &isect);

                if (cast_to_light (space, &tmp_origin, &tolight,
                                   offset_magtolight))
                {
                    real intensity;
                    intensity = light->intensity;
                        /* intensity *= 1 / (magtolight * magtolight); */

                        /* Add diffuse portion.*/
                    dscale += tscale * intensity;

                        /* Specular */
                    if (material)
                    {
                        real dot;
                        dot = dot_Point (&refldir, &tolight);
                        if (dot > 0)
                        {
                            tscale = pow (dot, material->shininess);
                            sscale += tscale;
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

            tscale = ambient + diffuse * dscale + specular * sscale;
            if (material && material->opacity > 0)
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

            scale_Point (&tmp_origin, &tmp_dir, offset);
            summ_Point (&tmp_origin, &tmp_origin, &isect);
            cast_colors (colors, space, image, &tmp_origin, &tmp_dir,
                         factors, nbounces);
        }
        if (reflective)
        {
            Point tmp_origin;
            real factors[NColors];
            UFor( i, NColors )
                factors[i] = (material->opacity * material->specular[i]);
            scale_Point (&tmp_origin, &refldir, offset);
            summ_Point (&tmp_origin, &tmp_origin, &isect);
            cast_colors (colors, space, image, &tmp_origin, &refldir,
                         factors, nbounces);
        }
    }

    UFor( i, NColors )  ret_colors[i] += colors[i];
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

    hit_idx = *ret_hit;
    hit_mag = *ret_mag;

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
            return;
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

    cast_ray (&hit_idx, &hit_mag, origin, dir,
              space->main.nelems,
              space->main.tree.elemidcs,
              space->main.tree.nodes,
              space->main.simplices,
              space->main.elems,
              &space->main.box,
              inside_box);

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

        if (i == ignore_object)  continue;

        object = ray_to_ObjectRaySpace (&rel_origin, &rel_dir,
                                        origin, dir, space, i);

        rel_inside_box =
            inside_BoundingBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast_ray (&tmp_hit, &tmp_mag, &rel_origin, &rel_dir,
                  object->nelems, object->tree.elemidcs,
                  object->tree.nodes,
                  object->simplices, object->elems,
                  &object->box, rel_inside_box);

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
    bool
test_object_intersections (uint* ret_hit,
                           real* ret_mag,
                           uint* ret_object,
                           BitString* tested,
                           const Point* origin,
                           const Point* dir,
                           uint nobjectidcs,
                           const uint* objectidcs,
                           const RaySpace* space,
                           const BoundingBox* box)
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
                                        origin, dir, space, objidx);

        rel_inside_box =
            inside_BoundingBox (&object->box, &rel_origin);

        tmp_hit = Max_uint;
        tmp_mag = *ret_mag;
        cast_ray (&tmp_hit, &tmp_mag, &rel_origin, &rel_dir,
                  object->nelems, object->tree.elemidcs,
                  object->tree.nodes,
                  object->simplices, object->elems,
                  &object->box, rel_inside_box);

        if (tmp_mag < *ret_mag)
        {
            *ret_hit = tmp_hit;
            *ret_mag = tmp_mag;
            *ret_object = objidx;
        }
    }

    if (*ret_mag != Max_real)
    {
        Point isect;
        scale_Point (&isect, dir, *ret_mag);
        summ_Point (&isect, &isect, origin);
        return inside_BoundingBox (box, &isect);
    }
    return false;
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
    Point salo_entrance;
    uint node_idx, parent = 0;
    Point* restrict entrance;
    const KDTreeNode* restrict nodes;
    const uint* restrict elemidcs;
    uint hit_idx;
    real hit_mag;
    uint hit_object;

    assert (space->nobjects < ntestedbits);
    zero_BitString (tested, ntestedbits);
    if (ignore_object <= space->nobjects)
        set1_BitString (tested, ignore_object);

    entrance = &salo_entrance;
    nodes = space->object_tree.nodes;
    elemidcs = space->object_tree.elemidcs;

    hit_idx = *ret_hit;;
    hit_mag = *ret_mag;
    hit_object = *ret_object;

    if (inside_box)
    {
        const BoundingBox* box;
            /* Find the initial node.*/
        node_idx = find_KDTreeNode (&parent, origin, nodes);
        box = &nodes[node_idx].as.leaf.box;
        assert (inside_BoundingBox (box, origin));
    }
    else
    {
        if (! hit_outer_BoundingBox (entrance, &space->box, origin, dir))
            return;
        node_idx = 0;
    }

    do
    {
        __global const KDTreeLeaf* restrict leaf;
        bool hit_in_leaf;

        node_idx = descend_KDTreeNode (&parent, entrance, node_idx, nodes);

        leaf = &nodes[node_idx].as.leaf;
        hit_in_leaf =
            test_object_intersections (&hit_idx, &hit_mag, &hit_object,
                                       tested,
                                       origin, dir, leaf->nelems,
                                       &elemidcs[leaf->elemidcs],
                                       space, &leaf->box);
        if (hit_in_leaf)  break;

        node_idx = upnext_KDTreeNode (entrance, &parent,
                                      origin, dir, node_idx, nodes);
    }
    while (node_idx != parent);

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

        hitline = &hits[row * ncols];
        magline = &mags[row * ncols];

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
            cast_ray (&hit, &mag,
                      &origin, &dir,
                      object->nelems,
                      object->tree.elemidcs,
                      object->tree.nodes,
                      object->simplices,
                      object->elems,
                      box, inside_box);
            hitline[col] = hit;
            magline[col] = mag;

                /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
        }
    }
}


    void
setup_ray_pixel_deltas_orthographic (Point* dir_start,
                                     Point* row_delta,
                                     Point* col_delta,
                                     uint nrows, uint ncols,
                                     const Point* origin,
                                     const PointXfrm* view_basis,
                                     real view_width)
{
    const uint row_dim = 0, col_dim = 1;
    Point diff;
    real tstart, tdelta;

    copy_Point (dir_start, origin);

    tdelta = view_width / nrows;
    tstart = (- view_width + tdelta) / 2;

    scale_Point (row_delta, &view_basis->pts[row_dim], tdelta);
    scale_Point (&diff, &view_basis->pts[row_dim], tstart);
    summ_Point (dir_start, dir_start, &diff);

    tdelta = view_width / ncols;
    tstart = (- view_width + tdelta) / 2;

    scale_Point (col_delta, &view_basis->pts[col_dim], tdelta);
    scale_Point (&diff, &view_basis->pts[col_dim], tstart);
    summ_Point (dir_start, dir_start, &diff);
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

    if (image->hits)  hitline = &image->hits[row * ncols];
    if (image->mags)  magline = &image->mags[row * ncols];
    if (image->pixels)  pixline = &image->pixels[row * 3 * ncols];

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

    if (image->hits)  hitline = &image->hits[row * ncols];
    if (image->mags)  magline = &image->mags[row * ncols];
    if (image->pixels)  pixline = &image->pixels[row * 3 * ncols];

    origin = &known->origin;
    scale_Point (&partial_dir, &known->row_delta, row);
    summ_Point (&partial_dir, &partial_dir, &known->dir_start);

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

        scale_Point (&dir, &known->col_delta, col);
        summ_Point (&dir, &dir, &partial_dir);
        normalize_Point (&dir, &dir);

        cast_record (hitline, magline, pixline, col,
                     space, image,
                     origin, &dir, known->inside_box);

            /* if (row == 333 && col == 322)  puts (elem ? "hit" : "miss"); */
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
        copy_Point (&dst->origin, &view_basis->pts[DirDimension]);
        setup_ray_pixel_deltas_orthographic (&dst->dir_start,
                                             &dst->row_delta,
                                             &dst->col_delta,
                                             image->nrows, image->ncols,
                                             origin, view_basis,
                                             image->hifov);
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
        Point partial_ray_origin;
        copy_Point (dir, &known->origin);

        scale_Point (&partial_ray_origin, &known->row_delta, row);
        summ_Point (&partial_ray_origin, &partial_ray_origin, &known->dir_start);

        scale_Point (origin, &known->col_delta, col);
        summ_Point (origin, origin, &partial_ray_origin);
    }
    else
    {
        Point partial_dir;
        copy_Point (origin, &known->origin);

        scale_Point (&partial_dir, &known->row_delta, row);
        summ_Point (&partial_dir, &partial_dir, &known->dir_start);
        scale_Point (dir, &known->col_delta, col);
        summ_Point (dir, dir, &partial_dir);
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
    uint i, nprocs, myrank;

#ifdef TrivialMpiRayTrace
    MPI_Comm_size (MPI_COMM_WORLD, (int*) &nprocs);
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &myrank);
#else
    myrank = 0;
    nprocs = 1;
#endif

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (i = myrank; i < row_nul; i += nprocs)
    {
        if (image->perspective)
            cast_row_perspective (image, row_off + i, space, known);
        else
            cast_row_orthographic (image, row_off + i, space, known);
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

