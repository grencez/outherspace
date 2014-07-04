
#ifndef SERIAL_H_
#define SERIAL_H_
#include "cx/fileb.h"
#include "space.h"

void
oput_Point (OFile* of, const Point* x);
void
oput_PointXfrm (OFile* of, const PointXfrm* A);
void
oput_IAMap (OFile* of, const IAMap* A);

bool
xget_Point (XFile* xf, Point* x);
bool
xget_PointXfrm (XFile* xf, PointXfrm* A);
bool
xget_IAMap (XFile* xf, IAMap* A);

#include <stdio.h>

void output_Point (FILE* out, const Point* point);
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void output_Simplex (FILE* out, const Simplex* elem);
void output_BBox (FILE* out, const BBox* box);

#endif

