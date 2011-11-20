
#ifndef TRACK_H_
#define TRACK_H_
#include "scene.h"

typedef struct Track Track;

struct Track
{
    uint ncheckplanes;
    Plane* checkplanes;
    Point* checkpoints;

    uint nstartlocs;
    Point* startlocs;
    Point* startdirs;

    Scene scene;

    uint nmorphs;
    real* morph_dcoords;
    Scene* morph_scenes;
};

void
init_Track (Track* track);
void
cleanup_Track (Track* track);

#ifdef INCLUDE_SOURCE
#include "track.c"
#endif
#endif

