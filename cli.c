
#include "pnm-image.h"
#include "testcase.h"
#include "wavefront-file.h"
#include "radiosity.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <time.h>

int main (int argc, char** argv)
{
    const char pathname[] = "data";
    bool good = true;
    FILE* out;
    RaySpace space;
    RayImage image;
    Point view_origin;
    PointXfrm view_basis;
    bool write_image = true;
    real t0;
    Track track;

#ifdef DistribCompute
    init_compute (&argc, &argv);
#else
    (void) argc;
    (void) argv;
#endif

    out = stdout;
    init_Track (&track);
    init_RayImage (&image);

#if 1
    image.nrows = 2001;
    image.ncols = 2001;
    image.hits = AllocT( uint, 1 );
    image.mags = AllocT( real, 1 );
    image.pixels = AllocT( byte, 1 );
    good = setup_testcase_triangles (&space,
                                     &view_origin, &view_basis,
                                     &image.hifov,
                                     pathname);
#elif 1
    image.nrows = 1000;
    image.ncols = 1000;
    image.pixels = AllocT( byte, 1 );
    image.hifov = 60 * M_PI / 180;
    good = readin_Track (&track, &space, pathname, "curve-track.txt");
    if (good && track.nstartlocs > 0)
    {
        view_origin = track.startlocs[0];
        identity_PointXfrm (&view_basis);
        stable_orthorotate_PointXfrm (&view_basis, &view_basis,
                                      &track.startdirs[0], FwDim);
    }
#elif 0
    image.nrows = 1000;
    image.ncols = 1000;
    image.pixels = AllocT( byte, 1 );
    good = setup_testcase_simple (&space, &view_origin,
                                  &view_basis, &image.hifov,
                                  pathname, "machine0.obj");
#else
    image.nrows = 1500;
    image.ncols = 1500;
    image.pixels = AllocT( byte, 1 );
    good = setup_testcase_track
        (&space, &view_origin, &view_basis, &image.hifov,
         pathname);
#endif

    if (!good)
    {
        fputs ("Setup failed!\n", stderr);
        return 1;
    }

    resize_RayImage (&image);

        /* output_BBox (out, &space.box);  fputc ('\n', out); */
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
        rays_to_hits (&image, &space, &view_origin, &view_basis);
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
                                  &view_origin, &view_basis);
            printf ("sec:%f\n", monotime () - t0);
        }

        stop_computeloop ();
    }
#endif
#else
    update_dynamic_RaySpace (&space);
        /* cast_lights (&space, 2000, 4); */
    cast_RayImage (&image, &space, &view_origin, &view_basis);
    printf ("sec:%f\n", monotime () - t0);
#endif

    if (write_image)
    {
            /* downsample_RayImage (&image, 4); */
        unstride_RayImage (&image);
        if (image.hits)
        {
            output_PBM_image ("out.pbm", image.nrows, image.ncols,
                              image.hits, space.main.nelems);
            output_PGM_image ("out.pgm", image.nrows, image.ncols,
                              image.hits, space.main.nelems);
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
    lose_Track (&track);

#ifdef DistribCompute
    cleanup_compute ();
#endif
    return 0;
}

