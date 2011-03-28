
#include "main.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <time.h>

int main ()
{
    FILE* out;
    RaySpace space;
    Point view_origin;
    PointXfrm view_basis;

#ifdef DistribCompute
    init_compute ();
#endif

    out = stdout;
    zero_Point (&view_origin);

#if 1
    random_RaySpace (&space, 50);

    view_origin.coords[0] = 50;
    view_origin.coords[1] = 50;
    view_origin.coords[DirDimension] = -70;
    identity_PointXfrm (&view_basis);
#else
    {
        bool good = readin_wavefront (&space, "mba2.obj");
        if (!good)  return 1;
    }

    view_origin.coords[0] = 10;
    view_origin.coords[1] = 0;
    view_origin.coords[DirDimension] = -250;

    {
        PointXfrm tmp_basis;
        identity_PointXfrm (&tmp_basis);
            /* Tilt backwards a bit.*/
        tmp_basis.pts[0].coords[DirDimension] = -0.5;
        orthorotate_PointXfrm (&view_basis, &tmp_basis, 0);
    }

#endif

        /* output_BoundingBox (out, &space.scene.box); */

    build_KDTree (&space.tree, space.nelems, space.elems, &space.scene.box);

        /* output_KDTree (out, &space.tree, space.nelems, space.elems); */
        /* output_gv_KDTree (out, &space.tree); */

    {
        RayImage image;
        bool write_image = true;
        const uint nrows = 2000;
        const uint ncols = 2000;
        const real view_angle = M_PI / 3;

        init_RayImage (&image);
        image.hits = AllocT( uint, nrows * ncols );
        image.mags = AllocT( real, nrows * ncols );
            /* image.pixels = AllocT( byte, nrows * 3 * ncols ); */
        image.nrows = nrows;
        image.ncols = ncols;

#ifdef BENCHMARKING
        write_image = false;
#endif

#ifdef DistribCompute
        if (rays_to_hits_computeloop (&space))
        {
            write_image = false;
        }
        else
        {
            compute_rays_to_hits (&image, &space,
                                  &view_origin, &view_basis, view_angle);
            stop_computeloop ();
        }
#else
        rays_to_hits (&image, &space, &view_origin, &view_basis, view_angle);
#endif
        if (write_image)
        {
            if (image.hits)
            {
                output_PBM_image ("out.pbm", nrows, ncols,
                                  image.hits, space.nelems);
                output_PGM_image ("out.pgm", nrows, ncols,
                                  image.hits, space.nelems);
            }
            if (image.pixels)
            {
                output_PPM_image ("out.ppm", nrows, ncols,
                                  image.pixels);
            }
        }
        cleanup_RayImage (&image);
    }

    cleanup_RaySpace (&space);

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}

