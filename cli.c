
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
    init_RaySpace (&space);
    zero_Point (&view_origin);

#if 1
    random_RaySpace (&space, 50);

    view_origin.coords[0] = 50;
    view_origin.coords[1] = 50;
    view_origin.coords[DirDimension] = -70;
    identity_PointXfrm (&view_basis);
#else
    {
        bool good = readin_wavefront (&space, "track_1.obj");
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


    space.nobjects = 1;
    space.objects = AllocT( RaySpaceObject, space.nobjects );
    init_RaySpace (&space.objects[0].space);
    {
        bool good = readin_wavefront (&space.objects[0].space, "machine_1.obj");
        if (!good)  return 1;
    }
    zero_Point (&space.objects[0].centroid);
    identity_PointXfrm (&space.objects[0].orientation);
#endif

        /* output_BoundingBox (out, &space.scene.box); */

    partition_RaySpace (&space);
        /* output_KDTree (stderr, &space.tree, space.nelems, space.elems); */
        /* output_gv_KDTree (out, &space.tree); */

    {
        RayImage image;
        bool write_image = true;
        const uint nrows = 2000;
        const uint ncols = 2000;
        const real view_angle = M_PI / 3;
        real t0;

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
            uint i;
            UFor( i, 10 )
            {
                t0 = monotime ();
                compute_rays_to_hits (&image, &space,
                                      &view_origin, &view_basis, view_angle);
                printf ("sec:%f\n", monotime () - t0);
            }

            stop_computeloop ();
        }
#else
        t0 = monotime ();
        rays_to_hits (&image, &space, &view_origin, &view_basis, view_angle);
        printf ("sec:%f\n", monotime () - t0);
#endif

        if (write_image)
        {
                /* downsample_RayImage (&image, 9); */
            if (image.hits)
            {
                output_PBM_image ("out.pbm", image.nrows, image.ncols,
                                  image.hits, space.nelems);
                output_PGM_image ("out.pgm", image.nrows, image.ncols,
                                  image.hits, space.nelems);
            }
            if (image.pixels)
            {
                const bool write_binary = false;
                if (write_binary)
                {
                    FILE* out;
                    size_t nbytes, nwrote;
                    nbytes = 3 * image.nrows * image.ncols;
                    nwrote = nbytes + 1;
                    out = fopen ("out.bin", "w+");
                    if (out)
                        nwrote = fwrite (image.pixels, 1, nbytes, out);
                    if (nwrote != nbytes)
                        fputs ("Failed to write binary file, out.bin!\n",
                               stderr);
                    fclose (out);
                }

                output_PPM_image ("out.ppm", image.nrows, image.ncols,
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

