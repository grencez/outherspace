
#include "main.h"

int main ()
{
    FILE* out;
    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;

    out = stdout;
    random_RaySpace (&space, 20);
    
    zero_Point (&view_origin);
    view_origin.coords[0] = -5;
    view_origin.coords[1] = 50;
    view_origin.coords[2] = -100;

    identity_PointXfrm (&view_basis);

        /* output_KDTree (out, &space.tree, space.nelems, space.selems); */

    {
        uint* hits;
        const uint nrows = 50;
        const uint ncols = 50;
        hits = AllocT( uint, nrows * ncols );
        rays_to_hits (hits, nrows, ncols,
                      space.nelems, space.selems, &space.tree,
                      &view_origin, &view_basis);
        output_PBM_image ("out.pbm", nrows, ncols, hits, space.nelems);
        output_PGM_image ("out.pgm", nrows, ncols, hits, space.nelems);
        free (hits);
    }

    cleanup_RaySpace (&space);

    return 0;
}

