
#include "track.h"

    void
init_Track (Track* track)
{
    track->ncheckplanes = 0;
    track->nstartlocs = 0;
    init_Scene (&track->scene);
}

    void
cleanup_Track (Track* track)
{
    if (track->ncheckplanes > 0)
    {
        free (track->checkplanes);
        free (track->checkpoints);
    }
    if (track->nstartlocs > 0)
    {
        free (track->startlocs);
        free (track->startdirs);
    }
    cleanup_Scene (&track->scene);
}

