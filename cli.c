
#include "main.h"

int main ()
{
    FILE* out;
    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;

    out = stdout;
    random_RaySpace (&space, 20);
        /* random_RaySpace (&space, 2); */
    
    zero_Point (&view_origin);
    view_origin.coords[0] = 50;
    view_origin.coords[1] = 50;
    view_origin.coords[2] = -696;

    identity_PointXfrm (&view_basis);

        /* output_KDTree (out, &space.tree, space.nelems, space.selems); */

    {
        uint* hits;
        const uint nrows = 440;
        const uint ncols = 430;
        hits = AllocT( uint, nrows * ncols );
#if 0
#elif 0
        rays_to_hits_perspective (hits, nrows, ncols,
                                  space.nelems, space.selems, &space.tree,
                                  view_origin.coords[2]);
#elif 1
        rays_to_hits (hits, nrows, ncols,
                      space.nelems, space.selems, &space.tree,
                      &view_origin, &view_basis);
#endif
        output_PBM_image ("out.pbm", nrows, ncols, hits, space.nelems);
        output_PGM_image ("out.pgm", nrows, ncols, hits, space.nelems);
        free (hits);
    }

    cleanup_RaySpace (&space);

    return 0;
}

