
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

static void
dbgout_compute (const char* format, ...);

static uint
simple_checksum (uint nbytes, const byte* bytes);
static void
big_compute_send (uint nbytes, byte* bytes, uint proc);
static void
big_compute_recv (uint nbytes, byte* bytes, uint proc);


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
                           const PointXfrm* restrict view_basis)
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
        if (space->nobjects > 0)
        {
            ret = MPI_Send (space->objects,
                            space->nobjects * sizeof (ObjectRaySpace),
                            MPI_BYTE,
                            proc, StdMsgTag, MPI_COMM_WORLD);
            AssertStatus( ret, "" );
        }
    }
#ifdef TrivialMpiRayTrace
    cast_RayImage (image, space, origin, view_basis);
#else
    rays_to_hits_balancer (image);
#endif
}


    uint
simple_checksum (uint nbytes, const byte* bytes)
{
    uint i;
    uint x = 0;
    UFor( i, nbytes )
        x += bytes[i];
    return x;
}

#ifdef CompressBigCompute
static
    void
output_bytes (FILE* out, uint nbytes, const byte* bytes)
{
    uint i;
    char buf[2 * CompressBufferSize + 1];
    UFor( i, nbytes )
        sprintf (&buf[2*i], "%02x", bytes[i]);

    buf[2*nbytes] = '\n';
    i = fwrite (buf, (2 * nbytes + 1) * sizeof(char), 1, out);
    assert (i == 1);
}
#endif

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

        if (flush)
        {
            ret = MPI_Wait (req, &status);
            AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
            dbgout_compute ("|> %u flip_bufs[%u][:n]", proc, flip);
#endif
        }

        strm.avail_out = CompressBufferSize;
        strm.next_out = buf;
        ret = deflate (&strm, Z_FINISH);
        assert (ret != Z_STREAM_ERROR);

        n = CompressBufferSize - strm.avail_out;
        if (n == 0)  break;

        ret = MPI_Isend (buf, n, MPI_BYTE,
                         proc, flip, MPI_COMM_WORLD, req);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("-> %u flip_bufs[%u][:%u]  checksum:%u",
                        proc, flip, n, simple_checksum (n, buf));
            /* output_bytes (stderr, n, buf); */
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

    ret = MPI_Send (0, 0, MPI_BYTE, proc, flip, MPI_COMM_WORLD);
    AssertStatus( ret, "Sync sender" );
#ifdef DebugMpiRayTrace
    dbgout_compute ("|> %u flip_bufs[%u][:0]", proc, flip);
#endif

    deflateEnd (&strm);
#else  /*^ #ifdef CompressBigCompute ^*/
    ret = MPI_Send (bytes, nbytes, MPI_BYTE,
                    proc, StdMsgTag, MPI_COMM_WORLD);
    AssertStatus( ret, "" );
#endif
}


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
               proc, flip, MPI_COMM_WORLD,
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
                   proc, flip, MPI_COMM_WORLD,
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
        dbgout_compute ("<| %u flip_bufs[%u][:%u]  checksum:%u",
                        proc, flip, n, simple_checksum (n, buf));
            /* output_bytes (stderr, n, buf); */
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
        assert (ret >= 0);
        if (ret == Z_STREAM_END)  break;
    }

    assert (strm.total_out == nbytes);

    flip = (flip + 1) % 2;
    ret = MPI_Wait (&flip_reqs[flip], &status);
    AssertStatus( ret, "" );
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
                       const PointXfrm* restrict view_basis);

bool rays_to_hits_computeloop (RaySpace* restrict space)
{
    uint proc = 0;
    ObjectRaySpace* xfer_objects = 0;

    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &proc);
    xfer_objects = AllocT( ObjectRaySpace, space->nobjects );

    if (proc == 0)  return false;

    while (true)
    {
        int ret;
        uint i;
        MPI_Status status;
        RayImage image;
        Point origin;
        PointXfrm view_basis;

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
        if (space->nobjects > 0)
        {
            ret = MPI_Recv (xfer_objects,
                            space->nobjects * sizeof (ObjectRaySpace),
                            MPI_BYTE,
                        0, StdMsgTag, MPI_COMM_WORLD, &status);
            AssertStatus( ret, "" );
        }

        if (image.hits)  image.hits = AllocT( uint, 1 );
        if (image.mags)  image.mags = AllocT( real, 1 );
        if (image.pixels)  image.pixels = AllocT( byte, 1 );
        resize_RayImage (&image);

        UFor( i, space->nobjects )
        {
            copy_Point (&space->objects[i].centroid,
                        &xfer_objects[i].centroid);
            copy_PointXfrm (&space->objects[i].orientation,
                            &xfer_objects[i].orientation);
        }

        update_dynamic_RaySpace (space);
#ifdef TrivialMpiRayTrace
        cast_RayImage (&image, space, &origin, &view_basis);
#else
        rays_to_hits_computer (&image, space, &origin, &view_basis);
#endif
        cleanup_RayImage (&image);
    }

    if (xfer_objects)  free (xfer_objects);
    return true;
}



void rays_to_hits_computer (RayImage* restrict image,
                            const RaySpace* restrict space,
                            const Point* restrict origin,
                            const PointXfrm* restrict view_basis)
{
    MPI_Status status;
    uint i, row_off, row_nul;
    uint nrows, ncols;
    uint intl[2];
    MPI_Request recv_intl_req;
    RayCastAPriori known;
    uint nrows_computed = 0;
    uint* rows_computed;

    nrows = image->nrows;
    ncols = image->ncols;

    setup_RayCastAPriori (&known, image, origin, view_basis, &space->main.box);

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
        real t0, dt;

        MPI_Irecv (intl, 2, MPI_UNSIGNED,
                   0, StdMsgTag, MPI_COMM_WORLD, &recv_intl_req);
#ifdef DebugMpiRayTrace
        dbgout_compute ("<- 0 intl(? ?)");
#endif
        t0 = monotime ();
        UFor( i, row_nul )
            rows_computed[nrows_computed++] = row_off + i;

        cast_partial_RayImage (image, row_off, row_nul,
                               space, &known,
                               origin, view_basis);

        dt = monotime () - t0;

        MPI_Wait (&recv_intl_req, &status);
        AssertStatus( status.MPI_ERROR, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("<| 0 intl(%u %u)", intl[0], intl[1]);
#endif
        row_off = intl[0];
        row_nul = intl[1];
        if (row_nul > 0)
            MPI_Send (&dt, 1, MPI_real, 0, StdMsgTag, MPI_COMM_WORLD);
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("done!");
#endif

    i = 0;
    while (i < nrows_computed )
    {
        uint row, n;
        row = rows_computed[i];
        n = 1;

        ++ i;
        while (i < nrows_computed && rows_computed[i] - row == n)
        {
            ++ i;
            ++ n;
        }

#ifdef DebugMpiRayTrace
        dbgout_compute ("Sending rows:%u-%u", row, row + n - 1);
#endif
        if (image->hits)
            big_compute_send (n * ncols * sizeof(uint),
                              (byte*) &image->hits[row * ncols],
                              0);
        if (image->mags)
            big_compute_send (n * ncols * sizeof(real),
                              (byte*) &image->mags[row * ncols],
                              0);
        if (image->pixels)
            big_compute_send (n * 3 * ncols,
                              &image->pixels[row * 3 * ncols],
                              0);
    }

    if (rows_computed)  free (rows_computed);
}



    void
rays_to_hits_balancer (RayImage* image)
{
    struct compute_struct
    {
        uint intl[2];
        uint ncomplete; /* Completed rows.*/
        uint npending;
        real dt;
        real compute_time;
    };
    typedef struct compute_struct Compute;
    uint nrows_per_workunit;
    uint i, row;
    uint nrows, ncols;
    int ret;
    Compute* cpts;
    uint ncomplete = 0;
    real compute_time = Epsilon_real;
    real t0;
    real kcoeff;
    uint ncomputers;

    int* row_computers;
    MPI_Request* send_intl_reqs;
    MPI_Request* recv_progress_reqs;
    MPI_Status status;

    nrows = image->nrows;
    ncols = image->ncols;

    ncomputers = nprocs - 1;
        /* Do you believe in magic coefficients? */
    kcoeff = (real) ncomputers * (ncomputers - 1) / 2;
    nrows_per_workunit = 1 + (uint) ((real) nrows / (10 * ncomputers));

    row_computers = AllocT( int, nrows );
    cpts = AllocT( Compute, ncomputers );
    send_intl_reqs = AllocT( MPI_Request, ncomputers );
    recv_progress_reqs = AllocT( MPI_Request, ncomputers );

    t0 = monotime ();

    row = 0;
    UFor( i, ncomputers )
    {
        int proc;
        uint j, row_off, row_nul;

        proc = 1+i;

        row_off = row;
        row += nrows_per_workunit;
        if (row > nrows)  row = nrows;
        row_nul = row - row_off;

        cpts[i].intl[0] = row_off;
        cpts[i].intl[1] = row_nul;
        cpts[i].ncomplete = 0;
        cpts[i].npending = nrows_per_workunit;

        UFor( j, row_nul )
            row_computers[row_off + j] = proc;

        MPI_Isend (cpts[i].intl, 2, MPI_UNSIGNED,
                   proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
        dbgout_compute ("-> %d intl(%u %u)", proc, row_off, row_nul);
#endif
    }

    UFor( i, ncomputers )
    {
        ret = MPI_Wait (&send_intl_reqs[i], &status);
        AssertStatus( ret, "Wait on the first batch of sends" );
        if (cpts[i].intl[1] == 0)
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
            row_nul = nrows_per_workunit;
            row += row_nul;
            if (row > nrows)
            {
                row = nrows;
                row_nul = row - row_off;
            }

            cpts[i].intl[0] = row_off;
            cpts[i].intl[1] = row_nul;
            cpts[i].ncomplete += cpts[i].npending;
            cpts[i].npending = row_nul;
            cpts[i].compute_time = Epsilon_real;

            UFor( j, row_nul )
                row_computers[row_off + j] = proc;

            MPI_Irecv (&cpts[i].dt, 1, MPI_real,
                       proc, StdMsgTag, MPI_COMM_WORLD,
                       &recv_progress_reqs[i]);
#ifdef DebugMpiRayTrace
            dbgout_compute ("<- %d progress", proc);
#endif
            MPI_Isend (cpts[i].intl, 2, MPI_UNSIGNED,
                       proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
            dbgout_compute ("-> %d intl(%u %u)", proc, row_off, row_nul);
#endif
        }
    }

    while (row < nrows)
    {
        int proc;
        uint j, row_off, row_nul;
        real dt;

        ret = MPI_Waitany (ncomputers, recv_progress_reqs, (int*) &i, &status);
        AssertStatus( ret, "" );
        proc = i+1;
#ifdef DebugMpiRayTrace
        dbgout_compute ("<| %d progress", proc);
#endif
        row_off = cpts[i].intl[0];
        row_nul = cpts[i].intl[1];
        dt = cpts[i].dt;
        
        ncomplete += row_nul;
        compute_time += dt;
        cpts[i].compute_time += dt;

        ret = MPI_Wait (&send_intl_reqs[i], &status);
        AssertStatus( ret, "" );
#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %d intl(%u %u)", proc, row_off, row_nul);
#endif

        row_off = row;
        row_nul = 1 + (uint)
            ((real) (nrows - row) *
             (cpts[i].ncomplete * compute_time) /
             (ncomplete * cpts[i].compute_time * kcoeff));

        row += row_nul;
        if (row > nrows)
        {
            row = nrows;
            row_nul = row - row_off;
        }

        cpts[i].intl[0] = row_off;
        cpts[i].intl[1] = row_nul;
        cpts[i].ncomplete += cpts[i].npending;
        cpts[i].npending = row_nul;

        UFor( j, row_nul )
            row_computers[row_off + j] = proc;

        MPI_Irecv (&cpts[i].dt, 1, MPI_real,
                   proc, StdMsgTag, MPI_COMM_WORLD,
                   &recv_progress_reqs[i]);
#ifdef DebugMpiRayTrace
        dbgout_compute ("<- %d progress", proc);
#endif
        MPI_Isend (cpts[i].intl, 2, MPI_UNSIGNED,
                   proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
#ifdef DebugMpiRayTrace
        dbgout_compute ("-> %d intl(%u %u)", proc, row_off, row_nul);
#endif
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("Wrapping up!");
#endif

    ret = MPI_Waitall (ncomputers, recv_progress_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    ret = MPI_Waitall (ncomputers, send_intl_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    UFor( i, ncomputers )
    {
        int proc;
        proc = i+1;

#ifdef DebugMpiRayTrace
        dbgout_compute ("|> %d progress", proc);
        dbgout_compute ("|> %d intl(%u %u)", proc,
                        cpts[i].intl[0], cpts[i].intl[1]);
#endif

        if (cpts[i].intl[1] == 0)
        {
            send_intl_reqs[i] = MPI_REQUEST_NULL;
        }
        else
        {
            cpts[i].intl[0] = nrows;
            cpts[i].intl[1] = 0;
            MPI_Isend (cpts[i].intl, 2, MPI_UNSIGNED,
                       proc, StdMsgTag, MPI_COMM_WORLD, &send_intl_reqs[i]);
        }
    }
    ret = MPI_Waitall (ncomputers, send_intl_reqs, MPI_STATUSES_IGNORE);
    AssertStatus( ret, "" );
    UFor( i, ncomputers )
    {
        recv_progress_reqs[i] = MPI_REQUEST_NULL;
        send_intl_reqs[i] = MPI_REQUEST_NULL;
    }

#ifdef DebugMpiRayTrace
    dbgout_compute ("DONE!");
#endif

    UFor( i, ncomputers )
    {
        int proc;
        proc = 1+i;
        row = 0;
        while (row < nrows)
        {
            while (row < nrows && proc != row_computers[row])
                ++ row;

            if (row < nrows)
            {
                uint n = 1;

                while (row + n < nrows && proc == row_computers[row + n])
                    ++ n;

#ifdef DebugMpiRayTrace
                dbgout_compute ("Getting rows:%u-%u from process:%d",
                                row, row + n - 1, proc);
#endif
                if (image->hits)
                    big_compute_recv (n * ncols * sizeof(uint),
                                      (byte*) &image->hits[row * ncols],
                                      proc);
                if (image->mags)
                    big_compute_recv (n * ncols * sizeof(real),
                                      (byte*) &image->mags[row * ncols],
                                      proc);
                if (image->pixels)
                    big_compute_recv (n * 3 * ncols,
                                      &image->pixels[row * 3 * ncols],
                                      proc);
                row += n;
            }
        }
    }

    free (row_computers);
    free (cpts);
    free (send_intl_reqs);
    free (recv_progress_reqs);
}

