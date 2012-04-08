
#ifndef SERIAL_H_
#define SERIAL_H_
#include "cx/fileb.h"
#include "space.h"

void
dumpp_Point (FileB* f, const Point* x);
void
dumpp_PointXfrm (FileB* f, const PointXfrm* A);
void
dumpp_IAMap (FileB* f, const IAMap* A);

bool
load_Point (FileB* f, Point* x);
bool
load_PointXfrm (FileB* f, PointXfrm* A);
bool
load_IAMap (FileB* f, IAMap* A);

#include <stdio.h>

void output_Point (FILE* out, const Point* point);
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void output_Simplex (FILE* out, const Simplex* elem);
void output_BBox (FILE* out, const BBox* box);

#ifdef IncludeC
#include "serial.c"
#endif
#endif

