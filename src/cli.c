
#include "cx/syscx.h"
#include "cx/alphatab.h"
#include "cx/fileb.h"

#include "dynamic-setup.h"
#include "pnm-image.h"
#include "testcase.h"
#include "wavefront-file.h"
#include "lightcut.h"

#ifdef DistribCompute
#include "compute.h"
#endif

#include <time.h>

int main (int argc, char** argv)
{
  int argi = init_sysCx (&argc, &argv);
  DecloStack1( AlphaTab, xdirname, dflt_AlphaTab() );
  const char* pathname = "data";
  const char* infilename = "curve-track.txt";
  bool good = true;
  FILE* out;
  RaySpace space;
  RayImage image;
  Point view_origin;
  PointXfrm view_basis;
  bool write_image = true;
  real t0, t1;
  Track track;

#ifdef DistribCompute
  init_compute (&argc, &argv);
  push_losefn_sysCx (cleanup_compute);
#endif

  if (argi < argc) {
    pathname = 0;
    infilename = argv[argi];
  }

  {
    uint sepidx =
      pathname2_AlphaTab (xdirname, pathname, infilename);
    if (sepidx == 0) {
      pathname = "";
    }
    else {
      xdirname->s[sepidx-1] = '\0';
      pathname = xdirname->s;
    }
    infilename = &xdirname->s[sepidx];
  }
  DBog2( "opening dir:%s  file:%s", pathname, infilename );


    out = stdout;
    init_Track (&track);
    init_RayImage (&image);


    image.nrows = 800;
    image.ncols = 800;
    image.pixels = AllocT( byte, 1 );
    /* image.hits = AllocT( uint, 1 ); */
    /* image.mags = AllocT( real, 1 ); */
#if 0
        /* 2001 x 2001 */
    image.hits = AllocT( uint, 1 );
    image.mags = AllocT( real, 1 );
    good = setup_testcase_triangles (&space,
                                     &view_origin, &view_basis,
                                     &image.hifov,
                                     pathname);
#elif 1
        /* 1000 x 1000 */
    good = readin_Track (&track, &space, pathname, infilename);
    if (good)
    {
        view_origin = track.camloc.xlat;
        transpose_PointXfrm (&view_basis, &track.camloc.xfrm);
        image.nrows = track.nimgrows;
        image.ncols = track.nimgcols;
    }
#elif 0
        /* 1000 x 1000 */
    if (!infilename)  infilename = "machine0.obj";
    good = setup_testcase_simple (&space, &view_origin,
                                  &view_basis, &image.hifov,
                                  pathname, infilename);
#else
        /* 1500 x 1500 */
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
    t1 = monotime ();
    printf ("Kd-tree build sec:%f\n", t1 - t0);
    t0 = t1;

    if (track.nphotons > 0)
    {
        cast_lights (&space, track.nphotons, track.nbounces);
        t1 = monotime ();
        printf ("nvpls:%u  lightcut tree build sec:%f\n", (uint)(space.lightcuts.nodes.sz+1)/2, t1 - t0);
        t0 = t1;
    }
    cast_RayImage (&image, &space, &view_origin, &view_basis);
    t1 = monotime ();
    printf ("Render sec:%f\n", t1 - t0);
#endif

    if (image.hits) {
      TableT( uint2 ) holes;
      InitTable( holes );
      find_holes (&holes, &image, space.main.nelems);
      for (uint i = 0; i < holes.sz; ++i) {
        DBog2("miss: %u %u", holes.s[i].s[0], holes.s[i].s[1]);
      }
      LoseTable( holes );
    }

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

  lose_AlphaTab (xdirname);

  lose_sysCx ();
  return 0;
}

