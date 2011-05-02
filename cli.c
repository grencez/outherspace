
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
    init_RayImage (&image);

#if 1
    image.nrows = 2000;
    image.ncols = 2000;
    image.hits = AllocT( uint, image.nrows * image.ncols );
    image.mags = AllocT( real, image.nrows * image.ncols );
    good = setup_testcase_triangles (&space,
                                     &view_origin, &view_basis,
                                     &view_angle);
#else
    image.nrows = 1000;
    image.ncols = 1000;
    image.pixels = AllocT( byte, image.nrows * 3 * image.ncols );
    good =
#if 0
#elif 0
        setup_testcase_track
#elif 0
        setup_testcase_4d_surface
#elif 1
        setup_testcase_sphere
#endif
        (&space, &view_origin, &view_basis, &view_angle);
#endif

    if (!good)
    {
        fputs ("Setup failed!\n", stderr);
        return 1;
    }

        /* output_BoundingBox (out, &space.box);  fputc ('\n', out); */
        /* output_KDTree (out, &space.tree, space.nelems, space.elems); */
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

