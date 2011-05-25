
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

void init_compute (int* argc, char*** argv);
void cleanup_compute ();

void stop_computeloop ();

void compute_rays_to_hits (RayImage* image,
                           const RaySpace* restrict space,
                           const Point* restrict origin,
                           const PointXfrm* restrict view_basis);
bool rays_to_hits_computeloop (RaySpace* restrict space);

#ifdef INCLUDE_SOURCE
#include "compute.c"
#endif
#endif

