
#include "testcase.h"

#include "wavefront-file.h"

static void
random_Point (Point* p, const BoundingBox* box);
static Point*
random_Points (uint npts, const BoundingBox* box);
static void
random_Scene (Scene* scene, uint nelems, const BoundingBox* box);

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
        space->box.min_corner.coords[i] = 0;
        space->box.max_corner.coords[i] = 100;
    }
    random_Scene (&space->scene, 50, &space->box);
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
    return true;
}


    bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle)
{
    bool good;
    uint i;

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    space->nobjects = 10;
    space->objects = AllocT( RaySpaceObject, space->nobjects );
    UFor( i, space->nobjects )
    {
        init_RaySpace (&space->objects[i].space);
        good = readin_wavefront (&space->objects[i].space.scene,
                                 "machine_1.obj");
        if (!good)  return false;
        init_filled_RaySpace (space);
        zero_Point (&space->objects[i].centroid);
        space->objects[i].centroid.coords[0] = 75 * i + 300;
        identity_PointXfrm (&space->objects[i].orientation);
    }

    good = readin_wavefront (&space->scene, "track_1.obj");
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

    return good;
}


    bool
setup_testcase_sphere (RaySpace* space,
                       Point* view_origin,
                       PointXfrm* view_basis,
                       real* view_angle)
{
    bool good;
    uint nscenes = 2;
    Scene scenes[2];

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->scene, "sphere1.obj");
        if (!good)  return false;
    }
    else
    {
        assert (NDimensions == 4);

        good = readin_wavefront (&scenes[0], "sphere1.obj");
        if (!good)  return false;
        good = readin_wavefront (&scenes[1], "sphere2.obj");
        if (!good)  return false;

        interpolate_Scene (&space->scene, NDimensions-1, nscenes, scenes);
    }

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = 0.305251;
    view_basis->pts[0].coords[0] = -0.176537;
    view_basis->pts[0].coords[1] = -0.0143234;
    view_basis->pts[0].coords[2] = -0.98419;
    view_origin->coords[1] = 0.265131;
    view_basis->pts[1].coords[0] = 0.65388;
    view_basis->pts[1].coords[1] = -0.749081;
    view_basis->pts[1].coords[2] = -0.106387;
    view_origin->coords[2] = -0.0538335;
    view_basis->pts[2].coords[0] = -0.735714;
    view_basis->pts[2].coords[1] = -0.662323;
    view_basis->pts[2].coords[2] = 0.141606;

    if (NDimensions == 4)
    {
        swaprows_PointXfrm (view_basis, 2, 3);
        view_origin->coords[3] = 50;
    }
#endif

    return good;
}

    bool
setup_testcase_4d_surface (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle)
{
    bool good;
    uint nscenes = 2;
    Scene scenes[2];

    init_RaySpace (space);
    identity_PointXfrm (view_basis);
    zero_Point (view_origin);

    if (NDimensions == 3)
    {
        good = readin_wavefront (&space->scene, "plate1.obj");
        if (!good)  return false;
    }
    else
    {
        assert (NDimensions == 4);

        good = readin_wavefront (&scenes[0], "plate1.obj");
        if (!good)  return false;
        good = readin_wavefront (&scenes[1], "plate2.obj");
        if (!good)  return false;

        interpolate_Scene (&space->scene, NDimensions-1, nscenes, scenes);
    }

    init_filled_RaySpace (space);

#if 0
#elif 1
    *view_angle = 1.0472;
    view_origin->coords[0] = -21322.4;
    view_basis->pts[0].coords[0] = -0.0701899;
    view_basis->pts[0].coords[1] = -0.997534;
    view_basis->pts[0].coords[2] = 3.35915e-05;
    view_basis->pts[0].coords[3] = 0;
    view_origin->coords[1] = 1885.96;
    view_basis->pts[1].coords[0] = 0.00932495;
    view_basis->pts[1].coords[1] = -0.000689809;
    view_basis->pts[1].coords[2] = -0.999956;
    view_basis->pts[1].coords[3] = 0;
    view_origin->coords[2] = -343.056;
    view_basis->pts[2].coords[0] = 0.99749;
    view_basis->pts[2].coords[1] = -0.0701865;
    view_basis->pts[2].coords[2] = 0.00935037;
    view_basis->pts[2].coords[3] = 0;
    if (NDimensions == 4)
    {
        swaprows_PointXfrm (view_basis, 2, 3);
        view_origin->coords[3] = 7.5;
    }
#endif

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

