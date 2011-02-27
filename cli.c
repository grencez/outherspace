
#include "main.h"
#include <time.h>

int main ()
{
    FILE* out;
    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;

    out = stdout;
    zero_Point (&view_origin);

#if 1
    random_RaySpace (&space, 50);
    
    view_origin.coords[0] = 50;
    view_origin.coords[1] = 50;
    view_origin.coords[2] = -70;
    identity_PointXfrm (&view_basis);
#else
    {
        bool good = readin_wavefront (&space, "mba2.obj");
        if (!good)  return 1;
    }

    view_origin.coords[0] = 0;
    view_origin.coords[1] = 10;
    view_origin.coords[2] = -250;

    {
        PointXfrm tmp_basis;
        identity_PointXfrm (&tmp_basis);
        tmp_basis.pts[1].coords[2] = -0.5;  /* Tilt backwards a bit.*/
        orthorotate_PointXfrm (&view_basis, &tmp_basis, 1);
    }

#endif

        /* output_BoundingBox (out, &space.scene.box); */

    build_KDTree (&space.tree, space.nelems, space.elems, &space.scene.box);

        /* output_KDTree (out, &space.tree, space.nelems, space.elems); */

    {
        uint* hits;
        const uint nrows = 2000;
        const uint ncols = 2000;
        hits = AllocT( uint, nrows * ncols );
#if 0
#elif 0
        rays_to_hits_perspective (hits, nrows, ncols,
                                  &space,
                                  view_origin.coords[2]);
#elif 1
        rays_to_hits (hits, nrows, ncols,
                      &space, &view_origin, &view_basis);
#endif
#ifndef BENCHMARKING
        output_PBM_image ("out.pbm", nrows, ncols, hits, space.nelems);
        output_PGM_image ("out.pgm", nrows, ncols, hits, space.nelems);
#endif  /* #ifndef BENCHMARKING */
        free (hits);
    }

    cleanup_RaySpace (&space);

    return 0;
}

