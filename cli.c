
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
#else
    {
        bool good = readin_wavefront (&space, "sample.obj");
        if (!good)  return 1;
    }

    view_origin.coords[0] = 0;
    view_origin.coords[1] = 0;
    view_origin.coords[2] = -10;
#endif

    build_KDTree (&space.tree, space.nelems, space.elems, &space.scene.box);
    identity_PointXfrm (&view_basis);

    output_KDTree (out, &space.tree, space.nelems, space.elems);

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
#endif /* BENCHMARKING */
        free (hits);
    }

    cleanup_RaySpace (&space);

    return 0;
}

