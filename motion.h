
#ifndef MOTION_H_
#ifndef __OPENCL_VERSION__
#define MOTION_H_
#include "raytrace.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

struct object_motion_struct;
typedef struct object_motion_struct ObjectMotion;

#define N2DimRotations (NDimensions * (NDimensions - 1) / 2)

struct object_motion_struct
{
    real mass;
    Point veloc;
    tristate thrust;
    bool boost;
    real rots[N2DimRotations];
    bool collide;
    bool gravity;
    bool stabilize;
};

void
init_ObjectMotion (ObjectMotion* motion);
void
rotate_object (ObjectMotion* motion, uint xdim, uint ydim, real angle);
void
move_objects (RaySpace* space, ObjectMotion* motions, real dt);

#ifndef __OPENCL_VERSION__
#ifdef INCLUDE_SOURCE
#include "motion.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

