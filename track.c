
#include "track.h"
#include "util.h"

    void
init_Track (Track* track)
{
    track->ncheckplanes = 0;
    track->nstartlocs = 0;
    init_Scene (&track->scene);
    track->nmorphs = 0;
    track->morph_dcoords = 0;
    track->morph_scenes = 0;
}

    void
cleanup_Track (Track* track)
{
    uint i;
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
    if (track->nmorphs > 0)
    {
        UFor( i, track->nmorphs )
            cleanup_Scene (&track->morph_scenes[i]);
        free (track->morph_dcoords);
        free (track->morph_scenes);
    }
}

