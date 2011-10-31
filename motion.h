
#ifndef MOTION_H_
#ifndef __OPENCL_VERSION__
#define MOTION_H_
#include "raytrace.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

typedef struct ObjectMotion ObjectMotion;

#define DriftDim 3

#define N2DimRotations (NDimensions * (NDimensions - 1) / 2)

struct ObjectMotion
{
    real mass;
    Point veloc;
    real thrust[2];
    bool boost;
    real rots[N2DimRotations];
    bool collide;
    bool gravity;
    bool stabilize;

    bool flying;
    real hover_height;
    real escape_height;
    Point track_normal;
    uint laps;
    uint checkpoint_idx;
    bool lock_drift;
};

void
init_ObjectMotion (ObjectMotion* motion, const ObjectRaySpace* object);
void
rotate_object (ObjectMotion* motion, uint xdim, uint ydim, real angle);
void
move_objects (RaySpace* space, ObjectMotion* motions, real dt,
              uint ncheckplanes, const Plane* checkplanes);

#ifndef __OPENCL_VERSION__
#ifdef INCLUDE_SOURCE
#include "motion.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

