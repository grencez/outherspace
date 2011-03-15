
#include "main.h"

#ifdef DISTRIBUTE_COMPUTE
#include "compute.h"
#endif

#include <time.h>

int main ()
{
    FILE* out;
    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;

#ifdef DISTRIBUTE_COMPUTE
    init_compute ();
#endif

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
        /* output_gv_KDTree (out, &space.tree); */

    {
        uint* hits;
        real* mags;
        bool write_image = true;
        const uint nrows = 2000;
        const uint ncols = 2000;
        const real view_angle = 2 * M_PI / 3;

        hits = AllocT( uint, nrows * ncols );
        mags = AllocT( real, nrows * ncols );

#ifdef BENCHMARKING
        write_image = false;
#endif

#ifdef DISTRIBUTE_COMPUTE
        if (rays_to_hits_computeloop (&space))
        {
            write_image = false;
        }
        else
        {
            compute_rays_to_hits (hits, mags, nrows, ncols, &space,
                                  &view_origin, &view_basis, view_angle);
            stop_computeloop ();
        }
#else

#if 0
        rays_to_hits_perspective (hits, mags, nrows, ncols, &space,
                                  view_origin.coords[2]);
#else
        rays_to_hits (hits, mags, nrows, ncols,
                      &space, &view_origin, &view_basis, view_angle);
#endif
#endif

        if (write_image)
        {
            output_PBM_image ("out.pbm", nrows, ncols, hits, space.nelems);
            output_PGM_image ("out.pgm", nrows, ncols, hits, space.nelems);
        }
        free (hits);
        free (mags);
    }

    cleanup_RaySpace (&space);

#ifdef DISTRIBUTE_COMPUTE
    cleanup_compute ();
#endif
    return 0;
}

