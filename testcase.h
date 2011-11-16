
#ifndef TESTCASE_H_
#define TESTCASE_H_
#include "raytrace.h"

bool
setup_testcase_triangles (RaySpace* space,
                          Point* view_origin, PointXfrm* view_basis,
                          real* view_angle,
                          const char* pathname);
bool
setup_testcase_simple (RaySpace* space,
                       Point* view_origin, PointXfrm* view_basis,
                       real* view_angle,
                       const char* pathname,
                       const char* file);
bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle,
                      const char* pathname);
bool
setup_testcase_bouncethru (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname);
bool
setup_testcase_smoothsphere (RaySpace* space,
                             Point* view_origin,
                             PointXfrm* view_basis,
                             real* view_angle,
                             const char* pathname);
bool
setup_testcase_4d_normals (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname);
bool
setup_testcase_manual_interp (RaySpace* space,
                              Point* view_origin,
                              PointXfrm* view_basis,
                              real* view_angle,
                              const char* pathname);
bool
setup_testcase_sphere (RaySpace* space,
                       Point* view_origin,
                       PointXfrm* view_basis,
                       real* view_angle,
                       const char* pathname);
void
setup_checkplanes_4d_surface (uint* ret_nplanes, Plane** ret_checkplanes,
                              Point** ret_checkpoints);
bool
setup_testcase_4d_surface (RaySpace* space,
                           Point* view_origin,
                           PointXfrm* view_basis,
                           real* view_angle,
                           const char* pathname);
#ifdef INCLUDE_SOURCE
#include "testcase.c"
#endif
#endif

