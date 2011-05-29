
#include "testcase.h"

#include "wavefront-file.h"

static void
random_Point (Point* p, const BoundingBox* box);
static Point*
random_Points (uint npts, const BoundingBox* box);
static void
random_Scene (Scene* scene, uint nelems, const BoundingBox* box);
static void
setup_camera_light (RaySpace* space, const Point* origin);

    bool
setup_testcase_triangles (RaySpace* space,
                          Point* view_origin, PointXfrm* view_basis,
                          real* view_angle)
{
    uint i;
    const bool usual_view = true;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    UFor( i, NDimensions )
    {
        space->main.box.min_corner.coords[i] = 0;
        space->main.box.max_corner.coords[i] = 100;
    }
    random_Scene (&space->main.scene, 50, &space->main.box);
    init_filled_RaySpace (space);

    if (usual_view)
    {
        *view_angle = M_PI / 3;
        UFor( i, NDimensions )
            view_origin->coords[i] = 50;
        view_origin->coords[DirDimension] = -70;
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

static
    bool
interpolate_by_file (Scene* dst,
                     uint nscenes,
                     const char* const* filepaths,
                     const real* dcoords)
{
    bool good = true;
    uint i;
    Scene* scenes;

    assert (NDimensions ==  4);
    if (NDimensions != 4)  return false;

    scenes = AllocT( Scene, nscenes );

    UFor( i, nscenes )
    {
        uint j;
        good = readin_wavefront (&scenes[i], filepaths[i]);
        if (!good)  return false;
        UFor( j, scenes[i].nverts )
            scenes[i].verts[j].coords[NDimensions-1] = dcoords[i];
    }

    interpolate_Scene (dst, NDimensions-1, nscenes, scenes);

    if (scenes)  free (scenes);
    return good;
}

static
    bool
setup_racers (RaySpace* space)
{
    uint i;
    bool good = true;
    UFor( i, space->nobjects )
    {
        ObjectRaySpace* object;
        object = &space->objects[i];
        init_ObjectRaySpace (object);
        object->centroid.coords[0] = 75 * i + 300;

        if (NDimensions == 3)
        {
            good = readin_wavefront (&object->scene, "machine_1.obj");
            if (!good)  return false;
        }
        else
        {
            const char* const paths[2] = { "machine_1.obj", "machine_1.obj" };
            const real dcoords[2] = { -0.1, 0.1 };
            interpolate_by_file (&object->scene, 2, paths, dcoords);
            swaprows_PointXfrm (&object->orientation, 2, 3);
            object->centroid.coords[3] = .5;
        }
    }
    return good;
}


    bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle)
{
    bool good;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    space->nobjects = 10;
    space->objects = AllocT( ObjectRaySpace, space->nobjects );

    good = readin_wavefront (&space->main.scene, "track_1.obj");
    if (!good)  return false;

    good = setup_racers (space);
    if (!good)  return false;

    init_filled_RaySpace (space);

    *view_angle = 2 * M_PI / 3;
    view_origin->coords[0] = (real) 988.474;
    view_basis->pts[0].coords[0] = (real) 0.770512;
    view_basis->pts[0].coords[1] = (real) -0.428396;
    view_basis->pts[0].coords[2] = (real) -0.472005;
    view_origin->coords[1] = (real) 82.3796;
    view_basis->pts[1].coords[0] = (real) 0.0501207;
    view_basis->pts[1].coords[1] = (real) -0.697475;
    view_basis->pts[1].coords[2] = (real) 0.714854;
    view_origin->coords[2] = (real) 183.883;
    view_basis->pts[2].coords[0] = (real) -0.635452;
    view_basis->pts[2].coords[1] = (real) -0.57446;
    view_basis->pts[2].coords[2] = (real) -0.515942;

    setup_camera_light (space, view_origin);
    return good;
}

    bool
setup_testcase_manual_interp (RaySpace* space,
                              Point* view_origin,
                              PointXfrm* view_basis,
                              real* view_angle)
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
            elem->pts[j] = elems[i][j];
            copy_Point (&simplex.pts[j], &scene->verts[elem->pts[j]]);
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
                       real* view_angle)
{
    bool good;
    FILE* out = stderr;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->main.scene, "sphere1.obj");
        if (!good)  return false;
    }
    else
    {
        const uint nscenes = 2;
        const char* const paths[2] = { "sphere1.obj", "sphere2.obj" };
        const real dcoords[2] = { -0.001, 1.001 };
        good = interpolate_by_file (&space->main.scene,
                                    nscenes, paths, dcoords);
        if (!good)  return false;
    }

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
                           real* view_angle)
{
    bool good = true;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    space->nobjects = 10;
    space->objects = AllocT( ObjectRaySpace, space->nobjects );

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->main.scene, "sandbox.obj");
    }
    else
    {
        const char* const paths[2] = { "sandbox.obj", "sandbox_flat.obj" };
        const real dcoords[2] = { -0.001, 1.001 };
        good = interpolate_by_file (&space->main.scene, 2, paths, dcoords);
    }
    if (!good)  return false;

    good = setup_racers (space);
    if (!good)  return false;

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = -21322.4;
    view_basis->pts[0].coords[0] = -0.0701899;
    view_basis->pts[0].coords[1] = -0.997534;
    view_basis->pts[0].coords[2] = 3.35915e-05;
    view_origin->coords[1] = 1885.96;
    view_basis->pts[1].coords[0] = 0.00932495;
    view_basis->pts[1].coords[1] = -0.000689809;
    view_basis->pts[1].coords[2] = -0.999956;
    view_origin->coords[2] = -343.056;
    view_basis->pts[2].coords[0] = 0.99749;
    view_basis->pts[2].coords[1] = -0.0701865;
    view_basis->pts[2].coords[2] = 0.00935037;
    if (NDimensions == 4)
    {
        swaprows_PointXfrm (view_basis, 2, 3);
        view_origin->coords[3] = 0.5;
    }
#endif

    setup_camera_light (space, view_origin);

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
        lo = box->min_corner.coords[ci];
        hi = box->max_corner.coords[ci];
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
            scene->elems[i].pts[pi] = pi + offset;
    }
}


    void
setup_camera_light (RaySpace* space, const Point* origin)
{
    assert (space->nlights == 0);
    space->nlights = 1;
    space->lights = AllocT( PointLightSource, space->nlights );
    init_PointLightSource (&space->lights[0]);
    copy_Point (&space->lights[0].location, origin);
}

