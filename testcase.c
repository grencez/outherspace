
#include "testcase.h"

#include "wavefront-file.h"

static void
random_Point (Point* p, const BoundingBox* box);
static Point*
random_Points (uint npts, const BoundingBox* box);
static void
random_RaySpace (RaySpace* space, uint nelems);

    bool
setup_testcase_triangles (RaySpace* space,
                          Point* view_origin, PointXfrm* view_basis,
                          real* view_angle)
{
#if 1
    uint i;
    (void) view_basis;
    random_RaySpace (space, 50);
    *view_angle = M_PI / 3;
    UFor( i, NDimensions )
        view_origin->coords[i] = 50;
    view_origin->coords[DirDimension] = -70;
#else
        /* Paste custom starting parameters here!
         * (Obtained by pressing 'P' in the GUI.)
         */
#endif

    return true;
}


    bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle)
{
    bool good;
    uint i;

    good = readin_wavefront (space, "track_1.obj");
    if (!good)  return false;

    space->nobjects = 10;
    space->objects = AllocT( RaySpaceObject, space->nobjects );
    UFor( i, space->nobjects )
    {
        init_RaySpace (&space->objects[i].space);
        good = readin_wavefront (&space->objects[i].space, "machine_1.obj");
        if (!good)  return false;
        zero_Point (&space->objects[i].centroid);
        space->objects[i].centroid.coords[0] = 75 * i + 300;
        identity_PointXfrm (&space->objects[i].orientation);
    }

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


void random_RaySpace (RaySpace* space, uint nelems)
{
    uint i;
    BoundingBox* box;

    box = &space->scene.box;

    {
        unsigned seed = 1294785237;
            /* unsigned seed = 1294968341; */
            /* seed = time (0); */
        fprintf (stderr, "Using seed: %u\n", seed);
        srand (seed);
    }

    UFor( i, NDimensions )
    {
        box->min_corner.coords[i] = 0;
        box->max_corner.coords[i] = 100;
    }

    space->scene.nverts = NDimensions * nelems;
    space->scene.verts = random_Points (space->scene.nverts, box);

    space->nelems = nelems;
    space->scene.nelems = nelems;
    space->elems = AllocT( Triangle, nelems );
    space->scene.elems = AllocT( SceneElement, nelems );
    space->nobjects = 0;

    UFor( i, nelems )
    {
        uint pi, offset;
        offset = i * NDimensions;
        init_SceneElement (&space->scene.elems[i]);
        UFor( pi, NDimensions )
        {
            if (pi < NTrianglePoints)
                copy_Point (&space->elems[i].pts[pi],
                            &space->scene.verts[pi + offset]);
            space->scene.elems[i].pts[pi] = pi + offset;
        }
    }
    space->simplices = AllocT( BarySimplex, nelems );
    UFor( i, nelems )
    {
        PointXfrm raw;
        elem_Scene (&raw, &space->scene, i);
        init_BarySimplex (&space->simplices[i], &raw);
    }
}

