
#include "testcase.h"

#include "dynamic-setup.h"
#include "wavefront-file.h"

#include <assert.h>

static void
random_Point (Point* p, const BoundingBox* box);
static Point*
random_Points (uint npts, const BoundingBox* box);
static void
random_Scene (Scene* scene, uint nelems, const BoundingBox* box);
static void
setup_camera_light (RaySpace* space, const Point* origin);
static void
setup_box_lights (RaySpace* space,
                  const PointLightSource* light,
                  const BoundingBox* box);
static bool
add_sky_texture (RaySpace* space,
                 const char* pathname,
                 const char* filename);

    bool
setup_testcase_triangles (RaySpace* space,
                          Point* view_origin, PointXfrm* view_basis,
                          real* view_angle,
                          const char* pathname)
{
    uint i;
    const bool usual_view = true;

    (void) pathname;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    UFor( i, NDimensions )
    {
        space->main.box.min.coords[i] = 0;
        space->main.box.max.coords[i] = 100;
    }
    random_Scene (&space->main.scene, 50, &space->main.box);
    init_filled_RaySpace (space);

    if (usual_view)
    {
        *view_angle = M_PI / 3;
        UFor( i, NDimensions )
            view_origin->coords[i] = 50;
        view_origin->coords[ForwardDim] = -70;
    }
    else
    {
        /* Paste custom starting parameters here!
         * (Obtained by pressing 'P' in the GUI.)
         */
    }
    setup_camera_light (space, view_origin);
    return true;
}

    bool
setup_testcase_simple (RaySpace* space,
                       Point* view_origin, PointXfrm* view_basis,
                       real* view_angle,
                       const char* pathname,
                       const char* file)
{
    AffineMap map;
    bool good;
    Point v;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    identity_AffineMap (&map);

    good = readin_wavefront (&space->main.scene, &map, pathname, file);
    if (!good)  return false;
    condense_Scene (&space->main.scene);
    init_filled_RaySpace (space);

    *view_angle = 2 * M_PI / 3;
    copy_Point (view_origin, &space->main.box.max);

    diff_Point (&v,
                &space->main.box.min,
                &space->main.box.max);

    orthorotate_PointXfrm (view_basis, view_basis, &v, ForwardDim);

    setup_camera_light (space, view_origin);
    return good;
}

    void
add_1elem_Scene_RaySpace (RaySpace* space)
{
    uint i;
    ObjectRaySpace* object;

    i = space->nobjects;
    if (i == 0)  space->objects = 0;
    space->nobjects += 1;
    ResizeT( ObjectRaySpace, space->objects, space->nobjects );

    object = &space->objects[i];
    init_ObjectRaySpace (object);
    setup_1elem_Scene (&object->scene);
    init_trivial_ObjectRaySpace (object);
}

    bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle,
                      const char* pathname)
{
    AffineMap map;
    bool good;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    identity_AffineMap (&map);

    space->objects = AllocT( ObjectRaySpace, space->nobjects );

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->main.scene, &map, pathname,
#if 0
                                 "track1.obj"
#elif 0
                                 "cone-track-wave.obj"
#elif 0
                                 "figure8-twist.obj"
#elif 1
                                 "curve-track.obj"
#elif 0
                                 "bentlooptex.obj"
#elif 1
                                 "loop.obj"
#endif
                                );
    }
    else
    {
        const char* const fnames[5] = { "loop.obj", 0, 0, 0, "bentloop.obj" };
        const real dcoords[5] = { -0.001, .25, .5, .75, 1.001 };
        good = interpolate_by_file (&space->main.scene, 5,
                                    pathname, fnames, dcoords);
    }
    if (!good)  return false;
    condense_Scene (&space->main.scene);
    fixup_wavefront_Scene (&space->main.scene);

    if (true)
    {
        PointXfrm fix;
        identity_PointXfrm (&fix);
        scale_PointXfrm (&fix, &fix, 700);
        xfrm_Scene (&space->main.scene, &fix);
    }
    else if (false)
    {
        PointXfrm fix;
        fixup_wavefront_Scene (&space->main.scene);
        identity_PointXfrm (&fix);
        scale_PointXfrm (&fix, &fix, 20);
        xfrm_Scene (&space->main.scene, &fix);
    }

    good = add_sky_texture (space, pathname, "iras.png");
    if (!good)  return false;

    init_filled_RaySpace (space);

    *view_angle = 2 * M_PI / 3,
    view_origin->coords[0] = 108.474;
    view_origin->coords[1] = 82.3796;
    view_origin->coords[2] = 183.883;
    trrotate_PointXfrm (view_basis, ForwardDim, RightDim, M_PI);
    trrotate_PointXfrm (view_basis, UpDim, ForwardDim, M_PI/3);


    if (NDimensions == 4)
    {
        swaprows_PointXfrm (view_basis, 2, 3);
        view_origin->coords[3] = .5;
    }

#if 0
#elif 1
    {
        Point loc;
        centroid_BoundingBox (&loc, &space->main.box);
        loc.coords[UpDim] += 5000;
        setup_camera_light (space, &loc);
    }
    Op_s( real, NColors, space->lights[0].intensity , .5 );
        /* space->lights[0].intensity = 1e6; */
#else
    space->nlights = 2;
    space->lights = AllocT( PointLightSource, space->nlights );
    copy_Point (&space->lights[0].location, view_origin);
    copy_Point (&space->lights[1].location, &space->main.box.max);
    Op_s( real, NColors, space->lights[0].intensity , .5 );
    Op_s( real, NColors, space->lights[1].intensity , .1 );
#endif
    return good;
}

    bool
setup_testcase_bouncethru (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname)
{
    AffineMap map;
    bool good;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);
    assert (NDimensions == 3);

    identity_AffineMap (&map);

    good = readin_wavefront (&space->main.scene, &map,
                             pathname, "reflection.obj");
    if (!good)  return false;
    fixup_wavefront_Scene (&space->main.scene);

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = -32.8964;
    view_basis->pts[0].coords[0] = 0.588301;
    view_basis->pts[0].coords[1] = 0.808218;
    view_basis->pts[0].coords[2] = 0.026177;
    view_origin->coords[1] = 25.5876;
    view_basis->pts[1].coords[0] = 0.121845;
    view_basis->pts[1].coords[1] = -0.1206;
    view_basis->pts[1].coords[2] = 0.985195;
    view_origin->coords[2] = 50.9825;
    view_basis->pts[2].coords[0] = 0.79941;
    view_basis->pts[2].coords[1] = -0.576402;
    view_basis->pts[2].coords[2] = -0.169426;
#endif

    space->nlights = 1;
    space->lights = AllocT( PointLightSource, space->nlights );
    init_PointLightSource (&space->lights[0]);
    space->lights[0].location.coords[0] = -61.2664;
    space->lights[0].location.coords[1] = 51.928;
    space->lights[0].location.coords[2] = -84.2657;
    Op_s( real, NColors, space->lights[0].intensity , .5 );

    return good;
}

    bool
setup_testcase_smoothsphere (RaySpace* space,
                             Point* view_origin,
                             PointXfrm* view_basis,
                             real* view_angle,
                             const char* pathname)
{
    AffineMap map;
    uint i;
    bool good;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);
    assert (NDimensions == 3);

    identity_AffineMap (&map);

    good = readin_wavefront (&space->main.scene, &map,
                             pathname, "sphere.obj");
    if (!good)  return false;
    fixup_wavefront_Scene (&space->main.scene);

    space->nobjects = 3;
    space->objects = AllocT( ObjectRaySpace, space->nobjects );
    UFor( i, space->nobjects )
    {
        ObjectRaySpace* object;
        object = &space->objects[i];
        init_ObjectRaySpace (object);
        object->centroid.coords[0] = 40 * (i+1);

        if (NDimensions == 3)
        {
            good = readin_wavefront (&object->scene, &map,
                                     pathname, "sphere.obj");
            if (!good)  return false;
            fixup_wavefront_Scene (&object->scene);
        }
    }

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = -45.0807;
    view_basis->pts[0].coords[0] = 0.286961;
    view_basis->pts[0].coords[1] = 0.343287;
    view_basis->pts[0].coords[2] = 0.894319;
    view_origin->coords[1] = 27.6182;
    view_basis->pts[1].coords[0] = -0.362903;
    view_basis->pts[1].coords[1] = -0.82504;
    view_basis->pts[1].coords[2] = 0.433139;
    view_origin->coords[2] = 6.87812;
    view_basis->pts[2].coords[0] = 0.886541;
    view_basis->pts[2].coords[1] = -0.448845;
    view_basis->pts[2].coords[2] = -0.112174;
#endif

    space->nlights = 1;
    space->lights = AllocT( PointLightSource, space->nlights );
    init_PointLightSource (&space->lights[0]);
    space->lights[0].location.coords[0] = -61.2664;
    space->lights[0].location.coords[1] = 51.928;
    space->lights[0].location.coords[2] = -84.2657;
    Op_s( real, NColors, space->lights[0].intensity , 1 );

    return good;
}


    bool
setup_testcase_4d_normals (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname)
{
#if NDimensions != 4
    (void) space;
    (void) view_origin;
    (void) view_basis;
    (void) view_angle;
    (void) pathname;
    assert (0);
    return false;
#else
    const Point verts[8] =
    {
        {{ 1, 0, 0, 0 }},
        {{ 0, 1, 0, 0 }},
        {{ 0, 0, 1, 0 }},
        {{ 0, 0, 0, 1 }},

        {{ 4, 1, 0, 0 }},
        {{ 4, 0, 1, 0 }},
        {{ 4, 0, 0, 1 }},
        {{ 5, 1, 1, 1 }}
    };
    const uint vertidcs[2][4] =
    {
        { 0, 1, 2, 3 },
        { 4, 5, 6, 7 }
    };
#if 0
    const Point salo_origin =
    {
            /* { -2, 0, 0, 0 } */
        { 1.51, 0.54, 0.99, 0.50 }
    };
    const PointXfrm salo_basis =
    {{
             /* {{ 1, 0, 0, 0 }}, */
             /* {{ 0, 1, 0, 0 }}, */
             /* {{ 0, 0, 0, 1 }}, */
             /* {{ 0, 0, 1, 0 }} */
         {{ 0.51, -0.72, -0.46, 0.00 }},
         {{ -0.22, -0.63, 0.74, 0.00 }},
         {{ 0.00, 0.00, 0.00, 1.00 }},
         {{ -0.83, -0.28, -0.49, 0.00}}
     }};
#else
    const Point salo_origin =
    {{ -0.598540, 1.404100, 0.886663, 1.694650 }};
    const PointXfrm salo_basis =
    {{
         {{ -0.102190, 0.589201, 0.519394, -0.610434 }},
         {{ 0.851197, 0.252687, 0.295333, 0.352691 }},
         {{ -0.103641, -0.631010, 0.766442, 0.060424 }},
         {{ 0.504262, -0.436827, -0.235741, -0.706632 }}
     }};
#endif
    uint i;
    Scene* scene;
    (void) pathname;

    *view_angle = 1.0472;
    copy_Point (view_origin, &salo_origin);
    copy_PointXfrm (view_basis, &salo_basis);

    init_RaySpace (space);
    scene = &space->main.scene;
    scene->nelems = 2;
    scene->elems = AllocT( SceneElement, scene->nelems );
    UFor( i, scene->nelems )
    {
        uint j;
        init_SceneElement (&scene->elems[i]);
        UFor( j, NDimensions )
            scene->elems[i].verts[j] = vertidcs[i][j];
    }

    scene->nverts = 8;
    scene->verts = AllocT( Point, scene->nverts );
    UFor( i, scene->nverts )
        copy_Point (&scene->verts[i], &verts[i]);
        
    scene->nmatls = 2;
    scene->matls = AllocT( Material, scene->nmatls );
    UFor( i, scene->nmatls )
        init_Material (&scene->matls[i]);

    scene->elems[0].material = 0;
    UFor( i, NColors )
        scene->matls[0].specular[i] = 1;
    scene->matls[0].diffuse[0] = .2;
    scene->matls[0].diffuse[1] = .2;
    scene->matls[0].diffuse[2] = .2;
    scene->matls[0].shininess = 15;

    scene->elems[1].material = 1;
    scene->matls[1].diffuse[0] = 1;
    scene->matls[1].diffuse[1] = .0;
    scene->matls[1].diffuse[2] = .0;

    setup_camera_light (space, view_origin);
        /* space->lights[0].intensity = 2; */

    init_filled_RaySpace (space);

    return true;
#endif
}


    bool
setup_testcase_manual_interp (RaySpace* space,
                              Point* view_origin,
                              PointXfrm* view_basis,
                              real* view_angle,
                              const char* pathname)
{
    FILE* out;
    uint i;
    const uint nverts = 6;
    const uint nelems = 15;
    const uint ndims = 4;
    uint elem_idx = 0;
    const real verts[][4] =
    {
            /* 0 */
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
            /* 3 */
        {0, 1, 0, 1},
        {1, 2, 0, 1},
        {0, 0, 1, 1},
    };
    const uint elems[][4] =
    {
        { 0, 1, 2,  3       },
        { 0, 1, 2,     4    },
        { 0, 1, 2,        5 },
        { 0, 1,     3, 4    },
        { 0, 1,     3,    5 },
        { 0, 1,        4, 5 },
        { 0,    2,  3, 4    },
        { 0,    2,  3,    5 },
        { 0,    2,     4, 5 },
        { 0,        3, 4, 5 },
        {    1, 2,  3, 4    },
        {    1, 2,  3,    5 },
        {    1, 2,     4, 5 },
        {    1,     3, 4, 5 },
        {       2,  3, 4, 5 },

        { 0, 0, 0, 0 }
    };
    Scene* scene;

    (void) pathname;

    assert (NDimensions == ndims);

    out = stderr;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    scene = &space->main.scene;
    scene->nelems = nelems;
    scene->nverts = nverts;
    scene->elems = AllocT( SceneElement, nelems );
    scene->verts = AllocT( Point, nverts );

    UFor( i, nverts )
    {
        uint j;
        UFor( j, ndims )
            scene->verts[i].coords[j] = verts[i][j];
    }

    UFor( i, nelems )
    {
        uint j;
        Simplex simplex;
        SceneElement* elem;

        elem = &scene->elems[elem_idx];
        init_SceneElement (elem);

        UFor( j, ndims )
        {
            elem->verts[j] = elems[i][j];
            copy_Point (&simplex.pts[j], &scene->verts[elem->verts[j]]);
        }

        if (degenerate_Simplex (&simplex))
        {
            fputs ("Skipping: ", out);
            output_Simplex (out, &simplex);
            fputc ('\n', out);
        }
        else
        {
            elem_idx += 1;
        }
    }
    scene->nelems = elem_idx;

    init_filled_RaySpace (space);

    *view_angle = 1.0472;
    swaprows_PointXfrm (view_basis, 2, 3);

#if 0
#elif 0
#endif

    setup_camera_light (space, view_origin);
    return true;
}

    bool
setup_testcase_sphere (RaySpace* space,
                       Point* view_origin,
                       PointXfrm* view_basis,
                       real* view_angle,
                       const char* pathname)
{
    AffineMap map;
    bool good;
    FILE* out = stderr;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    identity_AffineMap (&map);

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->main.scene, &map,
                                 pathname, "sphere1.obj");
    }
    else
    {
        const uint nscenes = 2;
        const char* const fnames[2] = { "sphere1.obj", "sphere2.obj" };
        const real dcoords[2] = { -0.001, 1.001 };
        good = interpolate_by_file (&space->main.scene,
                                    nscenes, pathname, fnames, dcoords);
    }
    if (!good)  return false;

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = 1.01034;
    view_basis->pts[0].coords[0] = -0.49957;
    view_basis->pts[0].coords[1] = 0.840569;
    view_basis->pts[0].coords[2] = 0.209461;
    view_origin->coords[1] = 0.648768;
    view_basis->pts[1].coords[0] = -0.708308;
    view_basis->pts[1].coords[1] = -0.257146;
    view_basis->pts[1].coords[2] = -0.657401;
    view_origin->coords[2] = -1.02315;
    view_basis->pts[2].coords[0] = -0.498728;
    view_basis->pts[2].coords[1] = -0.476781;
    view_basis->pts[2].coords[2] = 0.723844;


    if (NDimensions == 4)
    {
        swaprows_PointXfrm (view_basis, 2, 3);
        view_origin->coords[3] = 26.2109;
        view_origin->coords[3] = 0;
    }
#endif

    setup_camera_light (space, view_origin);
    output_BoundingBox (out, &space->main.box);
    return good;
}

    bool
setup_testcase_4d_surface (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname)
{
    AffineMap map;
    bool good = true;
    Point new_centroid;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    identity_AffineMap (&map);

    space->objects = AllocT( ObjectRaySpace, space->nobjects );

    zero_Point (&new_centroid);
    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->main.scene, &map,
                                 pathname, "sandbox.obj");
    }
    else
    {
        const char* const fnames[2] = { "sandbox_flat.obj", "sandbox.obj" };
        const real dcoords[2] = { -0.001, 1000.001 };
        good = interpolate_by_file (&space->main.scene, 2,
                                    pathname, fnames, dcoords);
        new_centroid.coords[3] = (dcoords[0] + dcoords[1]) / 2;
    }
    if (!good)  return false;

    condense_Scene (&space->main.scene);
    fixup_wavefront_Scene (&space->main.scene);
    recenter_Scene (&space->main.scene, &new_centroid);

    good = add_sky_texture (space, pathname, "iras.png");
    if (!good)  return false;

    init_filled_RaySpace (space);

#if 0
#elif 0
    *view_angle = 1.0472;
    view_origin->coords[0] = -4818.89;
    view_basis->pts[0].coords[0] = -0.987767;
    view_basis->pts[0].coords[1] = -0.146868;
    view_basis->pts[0].coords[2] = -0.0524107;
    view_origin->coords[1] = -2013.33;
    view_basis->pts[1].coords[0] = -0.0790953;
    view_basis->pts[1].coords[1] = 0.182214;
    view_basis->pts[1].coords[2] = 0.980073;
    view_origin->coords[2] = -1762.03;
    view_basis->pts[2].coords[0] = -0.134392;
    view_basis->pts[2].coords[1] = 0.972228;
    view_basis->pts[2].coords[2] = -0.191601;
#elif 1
    *view_angle = 2 * M_PI / 3;
    view_origin->coords[UpDim] = 200;
    view_origin->coords[RightDim] = 3000;
    view_origin->coords[ForwardDim] = -4000;
    if (NDimensions == 4)
        view_origin->coords[3] = 700;
    rotate_PointXfrm (view_basis, ForwardDim, UpDim, M_PI/5);
#endif

    if (true)
    {
        Point p;
        zero_Point (&p);
        p.coords[0] = 5000;
        setup_camera_light (space, &p);
        Op_s( real, NColors, space->lights[0].intensity , .5 );
        space->lights[0].diffuse = true;
    }
    else
    {
        BoundingBox box;
        PointLightSource light;

        copy_BoundingBox (&box, &space->main.box);
        box.min.coords[UpDim] = box.max.coords[UpDim];
        box.min.coords[UpDim] += 1000;
        box.max.coords[UpDim] += 2000;

        init_PointLightSource (&light);
        Op_20s( real, NColors, light.intensity
                ,/, light.intensity
                ,   exp2_uint (NDimensions-1) );
        light.diffuse = true;

        setup_box_lights (space, &light, &box);
    }

    return good;
}


    void
random_Point (Point* p, const BoundingBox* box)
{
    uint ci;
#if 0
    const uint dim_cutoff = 3;
#else
    const uint dim_cutoff = NDimensions;
#endif
    UFor( ci, NDimensions )
    {
        real x, lo, hi;
        lo = box->min.coords[ci];
        hi = box->max.coords[ci];
        if (ci < dim_cutoff)
            x = lo + (hi - lo) * ((real) rand () / RAND_MAX);
        else
            x = 0;
            /* printf ("%f\n", x); */
        p->coords[ci] = x;
    }
}


    Point*
random_Points (uint npts, const BoundingBox* box)
{
    uint i;
    Point* pts;
    pts = AllocT( Point, npts );

    UFor( i, npts )
        random_Point (&pts[i], box);
    return pts;
}


    void
random_Scene (Scene* scene, uint nelems, const BoundingBox* box)
{
    uint i;
    uint seed = 1294785237;

            /* seed = time (0); */
    fprintf (stderr, "Using seed: %u\n", seed);
    srand (seed);

    scene->nverts = NDimensions * nelems;
    scene->verts = random_Points (scene->nverts, box);

    scene->nelems = nelems;
    scene->elems = AllocT( SceneElement, nelems );

    UFor( i, nelems )
    {
        uint pi, offset;
        offset = i * NDimensions;
        init_SceneElement (&scene->elems[i]);
        UFor( pi, NDimensions )
            scene->elems[i].verts[pi] = pi + offset;
    }
}


    void
setup_camera_light (RaySpace* space, const Point* origin)
{
    assert (space->nlights == 0);
    space->nlights = 2;
    space->lights = AllocT( PointLightSource, space->nlights );
    init_PointLightSource (&space->lights[0]);
    init_PointLightSource (&space->lights[1]);
    copy_Point (&space->lights[0].location, origin);
    copy_Point (&space->lights[1].location, origin);
    Op_s( real, NColors,space->lights[1].intensity , 0 );
}

    void
setup_box_lights (RaySpace* space,
                  const PointLightSource* light,
                  const BoundingBox* box)
{
    uint i, ndims = 0;
    uint dims[NDimensions];

    UFor( i, NDimensions )
    {
        if (box->min.coords[i] != box->max.coords[i])
        {
            dims[ndims] = i;
            ndims += 1;
        }
    }

    space->nlights = exp2_uint (ndims);
    space->lights = AllocT( PointLightSource, space->nlights );

    UFor( i, space->nlights )
    {
        uint di, flags;
        Point* loc;

        flags = i;
        loc = &space->lights[i].location;

        copy_PointLightSource (&space->lights[i], light);
        copy_Point (loc, &box->max);

        UFor( di, ndims )
        {
            uint dim;
            dim = dims[di];
            if (even_uint (flags))
                loc->coords[dim] = box->min.coords[dim];
            else
                loc->coords[dim] = box->max.coords[dim];
            flags >>= 1;
        }
    }
}

    bool
add_sky_texture (RaySpace* space,
                 const char* pathname,
                 const char* filename)
{
    bool good;
    uint i;
    Scene* scene;
    scene = &space->main.scene;
    i = scene->ntxtrs;
    if (i == 0)  scene->txtrs = 0;  /* Assure this is NULL.*/
    scene->ntxtrs = i+1;
    ResizeT( Texture, scene->txtrs, i+1 );
    good = readin_Texture (&scene->txtrs[i], pathname, filename);
    if (good)  space->skytxtr = i;
    return good;
}

