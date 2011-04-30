
#ifndef TESTCASE_H_
#define TESTCASE_H_
#include "raytrace.h"

bool
setup_testcase_triangles (RaySpace* space,
                          Point* view_origin, PointXfrm* view_basis,
                          real* view_angle);
bool
setup_testcase_track (RaySpace* space,
                      Point* view_origin, PointXfrm* view_basis,
                      real* view_angle);

#ifdef INCLUDE_SOURCE
#include "testcase.c"
#endif
#endif

