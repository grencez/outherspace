
#ifndef TRACK_H_
#define TRACK_H_
#include "raytrace.h"

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
};

void
init_Track (Track* track);
void
cleanup_Track (Track* track);
bool
readin_Track (Track* track, RaySpace* space,
              const char* pathname, const char* filename);
bool
readin_checkplanes (uint* ret_nplanes, Plane** ret_planes, Point** ret_points,
                    const char* pathname, const char* filename);
#ifdef INCLUDE_SOURCE
#include "track.c"
#endif
#endif

