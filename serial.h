
#ifndef SERIAL_H_
#define SERIAL_H_
#include "cx/fileb.h"
#include "space.h"

void
dumpp_Point (OFileB* of, const Point* x);
void
dumpp_PointXfrm (OFileB* of, const PointXfrm* A);
void
dumpp_IAMap (OFileB* of, const IAMap* A);

bool
load_Point (XFileB* xf, Point* x);
bool
load_PointXfrm (XFileB* xf, PointXfrm* A);
bool
load_IAMap (XFileB* xf, IAMap* A);

#include <stdio.h>

void output_Point (FILE* out, const Point* point);
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void output_Simplex (FILE* out, const Simplex* elem);
void output_BBox (FILE* out, const BBox* box);

#ifdef IncludeC
#include "serial.c"
#endif
#endif

