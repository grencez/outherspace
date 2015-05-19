
#include "track.h"
#include "affine.h"
#include "util.h"

    void
init_Track (Track* track)
{
    track->ncheckplanes = 0;
    InitTable( track->startlocs );
    identity_IAMap (&track->camloc);
    init_Scene (&track->scene);
    track->nmorphs = 0;
    track->morph_dcoords = 0;
    track->morph_scenes = 0;

    track->nphotons = 0;
    track->nbounces = 0;
    track->nimgrows = 800;
    track->nimgcols = 800;
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
    LoseTable( track->startlocs );
    cleanup_Scene (&track->scene);
    if (track->nmorphs > 0)
    {
        UFor( i, track->nmorphs )
            cleanup_Scene (&track->morph_scenes[i]);
        free (track->morph_dcoords);
        free (track->morph_scenes);
    }
}

