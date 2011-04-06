
#include "compute.h"
#include "raytrace.h"
#include <mpi.h>

    /* #define CompressBigCompute */

#ifdef CompressBigCompute
#include <zlib.h>
#endif

    /* #define DebugMpiRayTrace */

static uint nprocs = 0;

void init_compute ()
{
    MPI_Init (0, 0);
    MPI_Comm_size (MPI_COMM_WORLD, (int*) &nprocs);
}


void cleanup_compute ()
{
    MPI_Finalize ();
}


void stop_computeloop ()
{
    uint i;
    UFor( i, nprocs-1 )
    {
        int proc;
        proc = 1+i;
        MPI_Send (0, 0, MPI_INT, proc, StopLoopMsgTag, MPI_COMM_WORLD);
    }
}


static void rays_to_hits_balancer (RayImage* image);

void compute_rays_to_hits (RayImage* image,
                           const RaySpace* restrict space,
                           const Point* restrict origin,
                           const PointXfrm* restrict view_basis,
                           real view_angle)
{
    uint i;
    (void) space;
    UFor( i, nprocs -1 )
    {
        int proc;
        proc = 1+i;
        MPI_Send (0, 0, MPI_INT, proc, StartComputeMsgTag, MPI_COMM_WORLD);
        MPI_Send (image, 1 * sizeof (RayImage), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send ((void*) origin, 1 * sizeof (Point), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send ((void*) view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send (&view_angle, 1 * sizeof (real), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
    }
    rays_to_hits_balancer (image);
}


static
    void
big_compute_send (uint nbytes, byte* bytes, uint proc)
{
#ifdef CompressBigCompute
    byte buf[BUFSIZ];
    z_stream strm;
    uint i = 0;

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    deflateInit(&strm, Z_DEFAULT_COMPRESSION);

    strm.avail_in = nbytes;
    strm.next_in = bytes;

    while (i < nbytes)
    {
        uint n;
        strm.avail_out = BUFSIZ;
        strm.next_out = buf;
        deflate (&strm, Z_FINISH);

        n = BUFSIZ - strm.avail_out;
        MPI_Send (&n, 1, MPI_UNSIGNED,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send (buf, n, MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        i += n;
    }

    deflateEnd(&strm);
#else
    MPI_Send (bytes, nbytes, MPI_BYTE,
              proc, StdMsgTag, MPI_COMM_WORLD);
#endif
}


static
    void
big_compute_recv (uint nbytes, byte* bytes, uint proc)
{
    MPI_Status status;
#ifdef CompressBigCompute
    byte buf[BUFSIZ];
    z_stream strm;
    uint i = 0;

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    inflateInit(&strm);

    strm.avail_out = nbytes;
    strm.next_out = bytes;

    while (i < nbytes)
    {
        uint n;
        MPI_Recv (&n, 1, MPI_UNSIGNED, proc, StdMsgTag,
                  MPI_COMM_WORLD, &status);

        assert (n < BUFSIZ);

        MPI_Recv (buf, n, MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD, &status);

        strm.avail_in = n;
        strm.next_in = buf;

        inflate(&strm, Z_NO_FLUSH);

        i += n;
    }

    inflateEnd(&strm);
#else
    MPI_Recv (bytes, nbytes, MPI_BYTE,
              proc, StdMsgTag, MPI_COMM_WORLD, &status);
#endif
}


static void rays_to_hits_computer (RayImage* restrict image,
                                   const RaySpace* restrict space,
                                   const Point* restrict origin,
                                   const PointXfrm* restrict view_basis,
                                   real view_angle, int proc);

bool rays_to_hits_computeloop (const RaySpace* restrict space)
{
    uint proc = 0;
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &proc);

    if (proc == 0)  return false;

    while (true)
    {
        MPI_Status status;
        RayImage image;
        Point origin;
        PointXfrm view_basis;
        real view_angle;

        MPI_Recv (0, 0, MPI_INT, 0, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
        if (status.MPI_TAG == StopLoopMsgTag)  break;
        if (status.MPI_TAG != StartComputeMsgTag)
        {
            assert (0);
            break;
        }

        MPI_Recv (&image, 1 * sizeof (RayImage), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&origin, 1 * sizeof (Point), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&view_angle, 1 * sizeof (real), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);

        if (image.hits)  image.hits = AllocT( uint, 1 );
        if (image.mags)  image.mags = AllocT( real, 1 );
        if (image.pixels)  image.pixels = AllocT( byte, 1 );
        resize_RayImage (&image);
        rays_to_hits_computer (&image, space,
                               &origin, &view_basis, view_angle,
                               proc);
        cleanup_RayImage (&image);
    }
    return true;
}



void rays_to_hits_computer (RayImage* restrict image,
                            const RaySpace* restrict space,
                            const Point* restrict origin,
                            const PointXfrm* restrict view_basis,
                            real view_angle, int proc)
{
    MPI_Status status;
    uint i, row_off, row_nul;
    uint nrows, ncols;
    uint intl[2];
    MPI_Request recv_intl_req;
    bool inside_box;
    Point dir_start, row_delta, col_delta;
    const BoundingBox* restrict box;
    uint nrows_computed = 0;
    uint* rows_computed;

#ifndef DebugMpiRayTrace
    (void) proc;
#endif

    nrows = image->nrows;
    ncols = image->ncols;

    box = &space->scene.box;
    inside_box = inside_BoundingBox (box, origin);

    setup_ray_pixel_deltas (&dir_start, &row_delta, &col_delta,
                            nrows, ncols,
                            view_basis, view_angle);
    
    rows_computed = AllocT( uint, nrows );

    MPI_Recv (intl, 2, MPI_UNSIGNED, 0, StdMsgTag, MPI_COMM_WORLD, &status);
#ifdef DebugMpiRayTrace
    fprintf (stderr, "Process %d: ->intl(? ?)\n", proc);
    fprintf (stderr, "Process %d: |>intl(%u %u)\n", proc, intl[0], intl[1]);
#endif
    row_off = intl[0];
    row_nul = intl[1];

    while (row_nul > 0)
    {
        MPI_Irecv (intl, 2, MPI_UNSIGNED,
                   0, StdMsgTag, MPI_COMM_WORLD, &recv_intl_req);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "Process %d: ->intl(? ?)\n", proc);
#endif
        UFor( i, row_nul )
        {
            uint row;
            row = row_off + i;
            rows_computed[nrows_computed++] = row;
            rays_to_hits_row (image, row,
                              space, origin, view_basis,
                              &dir_start, &row_delta, &col_delta,
                              inside_box);
        }

        MPI_Wait (&recv_intl_req, &status);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "Process %d: |>intl(%u %u)\n", proc, intl[0], intl[1]);
#endif
        row_off = intl[0];
        row_nul = intl[1];
        if (row_nul > 0)
            MPI_Send (0, 0, MPI_INT, 0, StdMsgTag, MPI_COMM_WORLD);
    }

#ifdef DebugMpiRayTrace
    fprintf (stderr, "Process %d: done!\n", proc);
#endif

    UFor( i, nrows_computed )
    {
        uint row;
        row = rows_computed[i];
#ifdef DebugMpiRayTrace
        fprintf (stderr, "Process %d: Sending row:%u\n", proc, row);
#endif
        if (image->hits)
            MPI_Send (&image->hits[row * ncols],
                      ncols * sizeof(uint), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
        if (image->mags)
            MPI_Send (&image->mags[row * ncols],
                      ncols * sizeof(real), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
        if (image->pixels)
            big_compute_send (3 * ncols, &image->pixels[row * 3 * ncols], 0);
    }

    if (rows_computed)  free (rows_computed);
}



    void
rays_to_hits_balancer (RayImage* image)
{
    uint nrows_per_workunit;
    uint i, row;
    uint nrows, ncols;

    int* row_computers;
    uint* intls;
    MPI_Request* send_intl_reqs;
    MPI_Request* recv_progress_reqs;
    MPI_Status status;

    nrows = image->nrows;
    ncols = image->ncols;

    nrows_per_workunit = 1 + nrows / (10 * nprocs);

    row_computers = AllocT( int, nrows );
    intls = AllocT( uint, 2*(nprocs-1) );
    send_intl_reqs = AllocT( MPI_Request, nprocs-1 );
    recv_progress_reqs = AllocT( MPI_Request, nprocs-1 );

    row = 0;
    UFor( i, nprocs-1 )
    {
        int proc;
        uint j, row_off, row_nul;

        proc = 1+i;

        row_off = row;
        row += nrows_per_workunit;
        if (row > nrows)  row = nrows;
        row_nul = row - row_off;

        intls[2*i  ] = row_off;
        intls[2*i+1] = row_nul;

        UFor( j, row_nul )
            row_computers[row_off + j] = proc;

        MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                   proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d<-intls(%u %u)\n", proc, row_off, row_nul);
#endif
    }

    UFor( i, nprocs-1 )
    {
        MPI_Wait (&send_intl_reqs[i], &status);
        if (intls[2*i+1] == 0)
        {
            send_intl_reqs[i] = MPI_REQUEST_NULL;
            recv_progress_reqs[i] = MPI_REQUEST_NULL;
        }
        else
        {
            int proc;
            uint j, row_off, row_nul;

            proc = 1+i;

            row_off = row;
            row += nrows_per_workunit;
            if (row > nrows)  row = nrows;
            row_nul = row - row_off;

            intls[2*i  ] = row_off;
            intls[2*i+1] = row_nul;

            UFor( j, row_nul )
                row_computers[row_off + j] = proc;

            MPI_Irecv (0, 0, MPI_INT,
                       proc, StdMsgTag, MPI_COMM_WORLD,
                       &recv_progress_reqs[i]);
#ifdef DebugMpiRayTrace
            fprintf (stderr, "    %d->progress\n", proc);
#endif
            MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                       proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
            fprintf (stderr, "    %d<-intls(%u %u)\n", proc, row_off, row_nul);
#endif
        }
    }

    while (row < nrows)
    {
        int proc;
        uint j, row_off, row_nul;

        MPI_Waitany (nprocs-1, recv_progress_reqs, (int*) &i, &status);
        proc = i+1;
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d|>progress\n", proc);
#endif
        row_off = intls[2*i  ];
        row_nul = intls[2*i+1];

        MPI_Wait (&send_intl_reqs[i], &status);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d<|intls(%u %u)\n", proc, row_off, row_nul);
#endif

        row_off = row;
        row += nrows_per_workunit;
        if (row > nrows)  row = nrows;
        row_nul = row - row_off;

        intls[2*i  ] = row_off;
        intls[2*i+1] = row_nul;

        UFor( j, row_nul )
            row_computers[row_off + j] = proc;

        MPI_Irecv (0, 0, MPI_BYTE,
                   proc, StdMsgTag, MPI_COMM_WORLD,
                   &recv_progress_reqs[i]);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d->progress\n", proc);
#endif
        MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                   proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d<-intls(%u %u)\n", proc, row_off, row_nul);
#endif
    }

    MPI_Waitany (nprocs-1, recv_progress_reqs, (int*) &i, &status);
    while (i < nprocs-1)
    {
        int proc;
        uint row_off, row_nul;

        proc = i+1;
        row_off = intls[2*i  ];
        row_nul = intls[2*i+1];
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d|>progress\n", proc);
#endif

        MPI_Wait (&send_intl_reqs[i], &status);
#ifdef DebugMpiRayTrace
        fprintf (stderr, "    %d<|intls(%u %u)\n", proc, row_off, row_nul);
#endif

        recv_progress_reqs[i] = MPI_REQUEST_NULL;
        if (row_nul == 0)
        {
            send_intl_reqs[i] = MPI_REQUEST_NULL;
        }
        else
        {
            intls[2*i  ] = nrows;
            intls[2*i+1] = 0;
            MPI_Send (&intls[2*i], 2, MPI_UNSIGNED,
                      proc, StdMsgTag, MPI_COMM_WORLD);
        }
        MPI_Waitany (nprocs-1, recv_progress_reqs, (int*) &i, &status);
    }

#ifdef DebugMpiRayTrace
    fputs ("DONE!\n", stderr);
#endif

    UFor( i, nprocs-1)
    {
        int proc;
        proc = 1+i;
        UFor( row, nrows )
        {
#ifdef DebugMpiRayTrace
            fprintf (stderr, "    Getting row:%u from process:%d\n", row, proc);
#endif
            if (proc == row_computers[row])
            {
                if (image->hits)
                    MPI_Recv (&image->hits[row * ncols],
                              ncols * sizeof(uint), MPI_BYTE,
                              proc, StdMsgTag, MPI_COMM_WORLD, &status);
                if (image->mags)
                    MPI_Recv (&image->mags[row * ncols],
                              ncols * sizeof(real), MPI_BYTE,
                              proc, StdMsgTag, MPI_COMM_WORLD, &status);
                if (image->pixels)
                    big_compute_recv (3 * ncols,
                                      &image->pixels[row * 3 * ncols],
                                      proc);
            }
        }
    }

    free (row_computers);
    free (intls);
    free (send_intl_reqs);
    free (recv_progress_reqs);
}

