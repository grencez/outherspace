
#include "compute.h"
#include "raytrace.h"

#include <mpi.h>
#include <stdarg.h>

    /* #define DebugMpiRayTrace */

    /* Defined in Makefile.*/
#ifdef CompressBigCompute
#include <zlib.h>
#define CompressBufferSize (1 << 13)
#endif

static uint nprocs = 0;

static
void dbgout_compute (const char* format, ...);

void init_compute (int* argc, char*** argv)
{
    MPI_Init (argc, argv);
    MPI_Comm_size (MPI_COMM_WORLD, (int*) &nprocs);
        /* Initialize time.*/
    MPI_Barrier (MPI_COMM_WORLD);
    monotime ();
    dbgout_compute ("");
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

    void
dbgout_compute (const char* format, ...)
{
    static int proc = -1;
    va_list ap;
    int ret;
    uint off = 0;
    char buf[2048];

    if (proc < 0)
    {
        MPI_Comm_rank (MPI_COMM_WORLD, &proc);
        return;
    }

    ret = sprintf (&buf[off], "%07.5f - %d - ", monotime(), proc);
    assert (ret > 0);
    off += ret;

    va_start (ap, format);
    ret = vsprintf (&buf[off], format, ap);
    assert (ret > 0);
    off += ret;
    va_end (ap);

    buf[off++] = '\n';
    buf[off] = '\0';
    
    fputs (buf, stderr);
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
        int ret;
        int proc;
        proc = 1+i;
        ret = MPI_Send (0, 0, MPI_INT,
                        proc, StartComputeMsgTag, MPI_COMM_WORLD);
        AssertStatus( ret, "" );
        ret = MPI_Send (image, 1 * sizeof (RayImage), MPI_BYTE,
                        proc, StdMsgTag, MPI_COMM_WORLD);
        AssertStatus( ret, "" );
        ret = MPI_Send ((void*) origin, 1 * sizeof (Point), MPI_BYTE,
                        proc, StdMsgTag, MPI_COMM_WORLD);
        AssertStatus( ret, "" );
        ret = MPI_Send ((void*) view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                        proc, StdMsgTag, MPI_COMM_WORLD);
        AssertStatus( ret, "" );
        ret = MPI_Send (&view_angle, 1 * sizeof (real), MPI_BYTE,
                        proc, StdMsgTag, MPI_COMM_WORLD);
        AssertStatus( ret, "" );
    }
    rays_to_hits_balancer (image);
}


static
    void
big_compute_send (uint nbytes, byte* bytes, uint proc)
{
    int ret;
#ifdef CompressBigCompute
    MPI_Status status;
    bool flush = false;
    uint flip = 0;
    byte flip_bufs[2][CompressBufferSize];
    MPI_Request flip_reqs[2];
    z_stream strm;

    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    deflateInit (&strm, Z_DEFAULT_COMPRESSION);

    strm.avail_in = nbytes;
    strm.next_in = bytes;

    strm.avail_out = 0;
    while (strm.avail_out == 0)
    {
        int ret;
        byte* buf;
        MPI_Request* req;
        uint n;

        buf = flip_bufs[flip];
        req = &flip_reqs[flip];

        strm.avail_out = CompressBufferSize;
        strm.next_out = buf;
        ret = deflate (&strm, Z_FINISH);
        assert (ret != Z_STREAM_ERROR);

        n = CompressBufferSize - strm.avail_out;
        if (n == 0)  break;

        if (flush)
        {
            ret = MPI_Wait (req, &status);
            AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
            dbgout_compute ("|> %u flip_bufs[%u][:n]", proc, flip);
#endif
        }
        ret = MPI_Isend (buf, n, MPI_BYTE,
                         proc, StdMsgTag, MPI_COMM_WORLD, req);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("-> %u flip_bufs[%u][:%u]", proc, flip, n);
#endif

        flip = (flip + 1) % 2;
        if (flip == 0)  flush = true;
    }

    assert (strm.total_in == nbytes);

    if (flush || flip > 0)
    {
        ret = MPI_Wait (&flip_reqs[0], &status);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %u flip_bufs[0][:n]", proc);
#endif
    }
    if (flush || flip > 1)
    {
        ret = MPI_Wait (&flip_reqs[1], &status);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %u flip_bufs[1][:n]", proc);
#endif
    }

    ret = MPI_Send (0, 0, MPI_BYTE, proc, StdMsgTag, MPI_COMM_WORLD);
    AssertStatus( ret, "Sync sender" );
#ifdef DebugMpiRayTrace
    dbgout_compute ("|> %u flip_bufs[%u][:0]", proc, flip);
#endif

    deflateEnd (&strm);
#else
    ret = MPI_Send (bytes, nbytes, MPI_BYTE,
                    proc, StdMsgTag, MPI_COMM_WORLD);
    AssertStatus( ret, "" );
#endif
}


static
    void
big_compute_recv (uint nbytes, byte* bytes, uint proc)
{
    int ret;
    MPI_Status status;
#ifdef CompressBigCompute
    uint flip = 0;
    byte flip_bufs[2][CompressBufferSize];
    MPI_Request flip_reqs[2];
    z_stream strm;

    memset (&strm, 0, sizeof(z_stream));
    ret = inflateInit(&strm);
    AssertStatus( ret, "Init zlib inflate stream" );

    strm.avail_out = nbytes;
    strm.next_out = bytes;

    MPI_Irecv (flip_bufs[flip], CompressBufferSize, MPI_BYTE,
               proc, StdMsgTag, MPI_COMM_WORLD,
               &flip_reqs[flip]);
#ifdef DebugMpiRayTrace
    dbgout_compute ("<- %u flip_bufs[%u][:n]", proc, flip);
#endif

    flip = (flip + 1) % 2;
    while (true)
    {
        byte* buf;
        MPI_Request* req;
        uint n;

        MPI_Irecv (flip_bufs[flip], CompressBufferSize, MPI_BYTE,
                   proc, StdMsgTag, MPI_COMM_WORLD,
                   &flip_reqs[flip]);
#ifdef DebugMpiRayTrace
        dbgout_compute ("<- %u flip_bufs[%u][:n]", proc, flip);
#endif

        flip = (flip + 1) % 2;
        buf = flip_bufs[flip];
        req = &flip_reqs[flip];
        ret = MPI_Wait (req, &status);
        AssertStatus( ret, "Wait on async compression recv" );

        MPI_Get_count (&status, MPI_BYTE, (int*) &n);
#ifdef DebugMpiRayTrace
        dbgout_compute ("<| %u flip_bufs[%u][:%u]", proc, flip, n);
#endif
        assert (n <= CompressBufferSize);
        assert (n > 0);

        strm.avail_in = n;
        strm.next_in = buf;

        ret = inflate (&strm, Z_NO_FLUSH);
#ifdef DebugMpiRayTrace
        dbgout_compute ("inflate ret:%d", ret);
        dbgout_compute ("avail_out:%d", strm.avail_out);
#endif
        if (ret == Z_STREAM_END)  break;
    }

    assert (strm.total_out == nbytes);

    flip = (flip + 1) % 2;
    MPI_Wait (&flip_reqs[flip], &status);
#ifdef DebugMpiRayTrace
    dbgout_compute ("<| %u flip_bufs[%u][:0]", proc, flip);
#endif

    inflateEnd (&strm);
#else
    ret = MPI_Recv (bytes, nbytes, MPI_BYTE,
                    proc, StdMsgTag, MPI_COMM_WORLD, &status);
    AssertStatus( ret, "Recv large computation" );
#endif
}


static void
rays_to_hits_computer (RayImage* restrict image,
                       const RaySpace* restrict space,
                       const Point* restrict origin,
                       const PointXfrm* restrict view_basis,
                       real view_angle);

bool rays_to_hits_computeloop (const RaySpace* restrict space)
{
    uint proc = 0;
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &proc);

    if (proc == 0)  return false;

    while (true)
    {
        int ret;
        MPI_Status status;
        RayImage image;
        Point origin;
        PointXfrm view_basis;
        real view_angle;

        ret = MPI_Recv (0, 0, MPI_INT, 0, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
        AssertStatus( ret, "Recv compute instruction" );
        if (status.MPI_TAG == StopLoopMsgTag)  break;
        if (status.MPI_TAG != StartComputeMsgTag)
        {
            assert (0);
            break;
        }

        ret = MPI_Recv (&image, 1 * sizeof (RayImage), MPI_BYTE,
                        0, StdMsgTag, MPI_COMM_WORLD, &status);
        AssertStatus( ret, "" );
        ret = MPI_Recv (&origin, 1 * sizeof (Point), MPI_BYTE,
                        0, StdMsgTag, MPI_COMM_WORLD, &status);
        AssertStatus( ret, "" );
        ret = MPI_Recv (&view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                        0, StdMsgTag, MPI_COMM_WORLD, &status);
        AssertStatus( ret, "" );
        ret = MPI_Recv (&view_angle, 1 * sizeof (real), MPI_BYTE,
                        0, StdMsgTag, MPI_COMM_WORLD, &status);
        AssertStatus( ret, "" );

        if (image.hits)  image.hits = AllocT( uint, 1 );
        if (image.mags)  image.mags = AllocT( real, 1 );
        if (image.pixels)  image.pixels = AllocT( byte, 1 );
        resize_RayImage (&image);
        rays_to_hits_computer (&image, space,
                               &origin, &view_basis, view_angle);
        cleanup_RayImage (&image);
    }
    return true;
}



void rays_to_hits_computer (RayImage* restrict image,
                            const RaySpace* restrict space,
                            const Point* restrict origin,
                            const PointXfrm* restrict view_basis,
                            real view_angle)
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

    nrows = image->nrows;
    ncols = image->ncols;

    box = &space->scene.box;
    inside_box = inside_BoundingBox (box, origin);

    setup_ray_pixel_deltas (&dir_start, &row_delta, &col_delta,
                            nrows, ncols,
                            view_basis, view_angle);
    
    rows_computed = AllocT( uint, nrows );

    MPI_Recv (intl, 2, MPI_UNSIGNED, 0, StdMsgTag, MPI_COMM_WORLD, &status);
    AssertStatus( status.MPI_ERROR, "" );
#ifdef DebugMpiRayTrace
    dbgout_compute ("<- 0 intl(? ?)");
    dbgout_compute ("<| 0 intl(%u %u)", intl[0], intl[1]);
#endif
    row_off = intl[0];
    row_nul = intl[1];

    while (row_nul > 0)
    {
        MPI_Irecv (intl, 2, MPI_UNSIGNED,
                   0, StdMsgTag, MPI_COMM_WORLD, &recv_intl_req);
#ifdef DebugMpiRayTrace
        dbgout_compute ("<- 0 intl(? ?)");
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
        AssertStatus( status.MPI_ERROR, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("<| 0 intl(%u %u)", intl[0], intl[1]);
#endif
        row_off = intl[0];
        row_nul = intl[1];
        if (row_nul > 0)
            MPI_Send (0, 0, MPI_INT, 0, StdMsgTag, MPI_COMM_WORLD);
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("done!");
#endif

    UFor( i, nrows_computed )
    {
        uint row;
        row = rows_computed[i];
#ifdef DebugMpiRayTrace
        dbgout_compute ("Sending row:%u", row);
#endif
        if (image->hits)
            big_compute_send (ncols * sizeof(uint),
                              (byte*) &image->hits[row * 3 * ncols],
                              0);
        if (image->mags)
            big_compute_send (ncols * sizeof(real),
                              (byte*) &image->mags[row * 3 * ncols],
                              0);
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
    int ret;

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
        dbgout_compute ("-> %d intls(%u %u)", proc, row_off, row_nul);
#endif
    }

    UFor( i, nprocs-1 )
    {
        ret = MPI_Wait (&send_intl_reqs[i], &status);
        AssertStatus( ret, "Wait on the first batch of sends" );
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
            dbgout_compute ("<- %d progress", proc);
#endif
            MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                       proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
            dbgout_compute ("-> %d intls(%u %u)", proc, row_off, row_nul);
#endif
        }
    }

    while (row < nrows)
    {
        int proc;
        uint j, row_off, row_nul;

        ret = MPI_Waitany (nprocs-1, recv_progress_reqs, (int*) &i, &status);
        AssertStatus( ret, "" );
        proc = i+1;
#ifdef DebugMpiRayTrace
        dbgout_compute ("<| %d progress", proc);
#endif
        row_off = intls[2*i  ];
        row_nul = intls[2*i+1];

        ret = MPI_Wait (&send_intl_reqs[i], &status);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %d intls(%u %u)", proc, row_off, row_nul);
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
        dbgout_compute ("<- %d progress", proc);
#endif
        MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                   proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
        dbgout_compute ("-> %d intls(%u %u)", proc, row_off, row_nul);
#endif
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("Wrapping up!");
#endif

    ret = MPI_Waitall (nprocs-1, recv_progress_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    ret = MPI_Waitall (nprocs-1, send_intl_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    UFor( i, nprocs-1 )
    {
        int proc;
        uint row_off, row_nul;

        proc = i+1;
        row_off = intls[2*i  ];
        row_nul = intls[2*i+1];

#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %d progress", proc);
        dbgout_compute ("|> %d intls(%u %u)", proc, row_off, row_nul);
#endif

        if (row_nul == 0)
        {
            send_intl_reqs[i] = MPI_REQUEST_NULL;
        }
        else
        {
            intls[2*i  ] = nrows;
            intls[2*i+1] = 0;
            MPI_Isend (&intls[2*i], 2, MPI_UNSIGNED,
                       proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
        }
    }
    ret = MPI_Waitall (nprocs-1, send_intl_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    UFor( i, nprocs-1 )
    {
        recv_progress_reqs[i] = MPI_REQUEST_NULL;
        send_intl_reqs[i] = MPI_REQUEST_NULL;
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("DONE!");
#endif

    UFor( i, nprocs-1)
    {
        int proc;
        proc = 1+i;
        UFor( row, nrows )
        {
            if (proc == row_computers[row])
            {
#ifdef DebugMpiRayTrace
                dbgout_compute ("Getting row:%u from process:%d", row, proc);
#endif
                if (image->hits)
                    big_compute_recv (ncols * sizeof(uint),
                                      (byte*) &image->hits[row * ncols],
                                      proc);
                if (image->mags)
                    big_compute_recv (ncols * sizeof(real),
                                      (byte*) &image->mags[row * ncols],
                                      proc);
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

