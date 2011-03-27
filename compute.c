
#include "compute.h"
#include "raytrace.h"
#include <mpi.h>

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


static void rays_to_hits_balancer (uint* hits, real* mags,
                                   uint nrows, uint ncols);

void compute_rays_to_hits (uint* hits, real* mags,
                           uint nrows, uint ncols,
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
        MPI_Send (&nrows, 1, MPI_UNSIGNED,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send (&ncols, 1, MPI_UNSIGNED,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send ((void*) origin, 1 * sizeof (Point), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send ((void*) view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
        MPI_Send (&view_angle, 1 * sizeof (real), MPI_BYTE,
                  proc, StdMsgTag, MPI_COMM_WORLD);
    }
    rays_to_hits_balancer (hits, mags, nrows, ncols);
}


static void rays_to_hits_computer (uint* hits, real* mags,
                                   uint nrows, uint ncols,
                                   const RaySpace* restrict space,
                                   const Point* restrict origin,
                                   const PointXfrm* restrict view_basis,
                                   real view_angle, int proc);

bool rays_to_hits_computeloop (const RaySpace* restrict space)
{
    uint proc;
    uint prev_nrows = 0, prev_ncols = 0;
    uint* hits = 0;
    real* mags = 0;
    MPI_Comm_rank (MPI_COMM_WORLD, (int*) &proc);

    if (proc == 0)  return false;

    while (true)
    {
        MPI_Status status;
        uint nrows, ncols;
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

        MPI_Recv (&nrows, 1, MPI_UNSIGNED,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&ncols, 1, MPI_UNSIGNED,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&origin, 1 * sizeof (Point), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&view_basis, 1 * sizeof (PointXfrm), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);
        MPI_Recv (&view_angle, 1 * sizeof (real), MPI_BYTE,
                  0, StdMsgTag, MPI_COMM_WORLD, &status);

        if (prev_nrows != nrows || prev_ncols != ncols)
        {
            if (hits)  free (hits);
            if (mags)  free (mags);
            hits = AllocT( uint, nrows * ncols );
            mags = AllocT( real, nrows * ncols );
        }

        rays_to_hits_computer (hits, mags,
                               nrows, ncols, space,
                               &origin, &view_basis, view_angle,
                               proc);
    }
    if (hits)  free (hits);
    if (mags)  free (mags);
    return true;
}



void rays_to_hits_computer (uint* restrict hits,
                            real* restrict mags,
                            uint nrows, uint ncols,
                            const RaySpace* restrict space,
                            const Point* restrict origin,
                            const PointXfrm* restrict view_basis,
                            real view_angle, int proc)
{
    MPI_Status status;
    uint i, row_off, row_nul;
    uint intl[2];
    MPI_Request recv_intl_req;
    bool inside_box;
    Point dir_start, row_delta, col_delta;
    const BoundingBox* restrict box;

    box = &space->scene.box;
    inside_box = inside_BoundingBox (box, origin);

    setup_ray_pixel_deltas (&dir_start, &row_delta, &col_delta,
                            nrows, ncols,
                            view_basis, view_angle);
    
    UFor( i, nrows )
        hits[i * ncols] = space->nelems+1;

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
            uint row, col;
            uint* hitline;
            real* magline;
            Point partial_dir;

            row = row_off + i;

            hitline = &hits[row * ncols];
            magline = &mags[row * ncols];
            scale_Point (&partial_dir, &row_delta, row);
            summ_Point (&partial_dir, &partial_dir, &dir_start);

            UFor( col, ncols )
            {
                Point dir;
                uint hit; real mag;

                scale_Point (&dir, &col_delta, col);
                summ_Point (&dir, &dir, &partial_dir);
                normalize_Point (&dir, &dir);

                cast_ray (&hit, &mag, origin, &dir,
                          space->nelems, space->elems,
                          space->tree.elemidcs, space->tree.nodes,
                          &space->scene.box, inside_box);
                hitline[col] = hit;
                magline[col] = mag;
            }
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

    fprintf (stderr, "Process %d: done!\n", proc);

    UFor( i, nrows )
    {
        if (hits[i * ncols] <= space->nelems)
        {
#ifdef DebugMpiRayTrace
            fprintf (stderr, "Process %d: Sending row:%u\n", proc, i);
#endif
            MPI_Send (&hits[i * ncols], ncols * sizeof(uint), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
            MPI_Send (&mags[i * ncols], ncols * sizeof(real), MPI_BYTE,
                      0, StdMsgTag, MPI_COMM_WORLD);
        }
    }
}



    void
rays_to_hits_balancer (uint* hits, real* mags,
                       uint nrows, uint ncols)
{
    uint nrows_per_workunit;
    uint i, row;

    int* row_computers;
    uint* intls;
    MPI_Request* send_intl_reqs;
    MPI_Request* recv_progress_reqs;
    MPI_Status status;

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

    fputs ("DONE!\n", stderr);
    UFor( i, nprocs-1)
    {
        int proc;
        proc = 1+i;
        UFor( row, nrows )
        {
                /* int proc; */
                /* proc = row_computers[row]; */
                /* assert (proc > 0); */
#ifdef DebugMpiRayTrace
            fprintf (stderr, "    Getting row:%u from process:%d\n", row, proc);
#endif
            if (proc == row_computers[row])
            {
                MPI_Recv (&hits[row * ncols], ncols * sizeof(uint), MPI_BYTE,
                          proc, StdMsgTag, MPI_COMM_WORLD, &status);
                MPI_Recv (&mags[row * ncols], ncols * sizeof(real), MPI_BYTE,
                          proc, StdMsgTag, MPI_COMM_WORLD, &status);
            }
        }
    }

    free (row_computers);
    free (intls);
    free (send_intl_reqs);
    free (recv_progress_reqs);
}

