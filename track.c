
#include "track.h"
#include "affine.h"
#include "util.h"

    void
init_Track (Track* track)
{
    InitTable( Ray, track->startlocs );
    identity_IAMap (&track->camloc);
    init_Scene (&track->scene);
    track->nmorphs = 0;
    track->morph_dcoords = 0;
    track->morph_scenes = 0;
}

    void
lose_Track (Track* track)
{
    uint i;
    if (track->ncheckplanes > 0)
    {
        free (track->checkplanes);
        free (track->checkpoints);
    }
    LoseTable( Ray, track->startlocs );
    cleanup_Scene (&track->scene);
    if (track->nmorphs > 0)
    {
        UFor( i, track->nmorphs )
            cleanup_Scene (&track->morph_scenes[i]);
        free (track->morph_dcoords);
        free (track->morph_scenes);
    }
}

