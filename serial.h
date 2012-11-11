
#ifndef SERIAL_H_
#define SERIAL_H_
#include "cx/fileb.h"
#include "space.h"

void
oput_Point (OFileB* of, const Point* x);
void
oput_PointXfrm (OFileB* of, const PointXfrm* A);
void
oput_IAMap (OFileB* of, const IAMap* A);

bool
xget_Point (XFileB* xf, Point* x);
bool
xget_PointXfrm (XFileB* xf, PointXfrm* A);
bool
xget_IAMap (XFileB* xf, IAMap* A);

#include <stdio.h>

void output_Point (FILE* out, const Point* point);
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void output_Simplex (FILE* out, const Simplex* elem);
void output_BBox (FILE* out, const BBox* box);

#ifdef IncludeC
#include "serial.c"
#endif
#endif

