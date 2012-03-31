
#ifndef DYNAMIC_SETUP_H_
#define DYNAMIC_SETUP_H_
#include "raytrace.h"
#include "space.h"
#include "track.h"

bool
interpolate_by_file (Scene* dst,
                     uint nscenes,
                     const char* pathname,
                     const char* const* filenames,
                     const real* dcoords);
bool
add_racers (RaySpace* space, uint nracers, const Track* track,
            const char* pathname);
bool
readin_Track (Track* track, RaySpace* space,
              const char* pathname, const char* filename);
bool
readin_checkplanes (uint* ret_nplanes, Plane** ret_planes, Point** ret_points,
                    const IAMap* map,
                    const char* pathname, const char* filename);
bool
parse_coord_system (PointXfrm* a, const char* line);
void
setup_box_lights (RaySpace* space,
                  const PointLightSource* light,
                  const BBox* box);

#ifdef IncludeC
#include "dynamic-setup.c"
#endif
#endif

