
#include "pnm-image.h"
#include "testcase.h"
#include "wavefront-file.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <time.h>

int main (int argc, char** argv)
{
    bool good = true;
    FILE* out;
    RaySpace space;
    RayImage image;
    Point view_origin;
    PointXfrm view_basis;
    real view_angle = 2 * M_PI / 3;
    bool write_image = true;
    real t0;

#ifdef DistribCompute
    init_compute (&argc, &argv);
#else
    (void) argc;
    (void) argv;
#endif

    out = stdout;
    init_RaySpace (&space);
    init_RayImage (&image);
    zero_Point (&view_origin);
    identity_PointXfrm (&view_basis);

#if 0
#elif 1
    image.nrows = 2000;
    image.ncols = 2000;
    image.hits = AllocT( uint, image.nrows * image.ncols );
    image.mags = AllocT( real, image.nrows * image.ncols );
    good = setup_testcase_triangles (&space,
                                     &view_origin, &view_basis,
                                     &view_angle);
#elif 1
    image.nrows = 1000;
    image.ncols = 1000;
    image.pixels = AllocT( byte, image.nrows * 3 * image.ncols );
    good = setup_testcase_track (&space,
                                 &view_origin, &view_basis,
                                 &view_angle);
#else
    image.nrows = 2000;
    image.ncols = 2000;
    good = readin_wavefront (&space, "track_1.obj");

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
    image.pixels = AllocT( byte, image.nrows * 3 * image.ncols );
#endif

    if (!good)
    {
        fputs ("Setup failed!\n", stderr);
        return 1;
    }

        /* output_BoundingBox (out, &space.scene.box); */

    partition_RaySpace (&space);
        /* output_KDTree (stderr, &space.tree, space.nelems, space.elems); */
        /* output_gv_KDTree (out, &space.tree); */

#ifdef BENCHMARKING
    write_image = false;
#endif

    t0 = monotime ();
#ifdef DistribCompute
#if 0
    {
        int myrank;
        MPI_Comm_rank (MPI_COMM_WORLD, &myrank);
        if (myrank != 0)  write_image = false;
        rays_to_hits (&image, &space, &view_origin,
                      &view_basis, view_angle);
        if (myrank == 0)
            printf ("sec:%f\n", monotime () - t0);
    }
#else
    if (rays_to_hits_computeloop (&space))
    {
        write_image = false;
    }
    else
    {
        uint i;
        UFor( i, 1 )
        {
            t0 = monotime ();
            compute_rays_to_hits (&image, &space,
                                  &view_origin, &view_basis, view_angle);
            printf ("sec:%f\n", monotime () - t0);
        }

        stop_computeloop ();
    }
#endif
#else
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
            else
            {
                output_PPM_image ("out.ppm", image.nrows, image.ncols,
                                  image.pixels);
            }
        }
    }
    cleanup_RayImage (&image);

    cleanup_RaySpace (&space);

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}

