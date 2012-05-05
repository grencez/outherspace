
#ifndef TRACK_H_
#define TRACK_H_
#include "scene.h"
#include "cx/table.h"

typedef struct Track Track;

#ifndef Table_Ray
#define Table_Ray Table_Ray
DeclTableT( Ray, Ray );
#endif

struct Track
{
    uint ncheckplanes;
    Plane* checkplanes;
    Point* checkpoints;

    TableT( Ray ) startlocs;
    IAMap camloc;

    Scene scene;

    uint nmorphs;
    real* morph_dcoords;
    Scene* morph_scenes;

    uint nphotons;
    uint nbounces;
    uint nimgrows;
    uint nimgcols;
};

void
init_Track (Track* track);
void
lose_Track (Track* track);

#ifdef IncludeC
#include "track.c"
#endif
#endif

