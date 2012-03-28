
#ifndef SERIAL_H_
#define SERIAL_H_
#include "cx/fileb.h"
#include "space.h"

void
dumpp_Point (FileB* f, const Point* x);
void
dumpp_PointXfrm (FileB* f, const PointXfrm* A);

#include <stdio.h>

void output_Point (FILE* out, const Point* point);
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void output_Simplex (FILE* out, const Simplex* elem);
void output_BBox (FILE* out, const BBox* box);

#ifdef IncludeC
#include "serial.c"
#endif
#endif

