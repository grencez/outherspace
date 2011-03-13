
#ifndef COMPUTE_H_
#define COMPUTE_H_
#include "raytrace.h"

enum compute_msg_tag_enum
{
    StdMsgTag,
    StopLoopMsgTag,
    StartComputeMsgTag,
    NComputeMsgTags
};
typedef enum compute_msg_tag_enum ComputeMsgTag;

void init_compute ();
void cleanup_compute ();

void stop_computeloop ();

void compute_rays_to_hits (uint* hits, real* mags,
                           uint nrows, uint ncols,
                           const RaySpace* restrict space,
                           const Point* restrict origin,
                           const PointXfrm* restrict view_basis,
                           real view_angle);
bool rays_to_hits_computeloop (const RaySpace* restrict space);

#ifdef INCLUDE_SOURCE
#include "compute.c"
#endif
#endif

