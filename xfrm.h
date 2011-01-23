
#ifndef XFRM_H_
#define XFRM_H_
#include "space.h"

struct point_xfrm_struct
{
    Point pts[NDimensions];
};
typedef struct point_xfrm_struct PointXfrm;

void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
void zero_PointXfrm (PointXfrm* xfrm);
void copy_PointXfrm (PointXfrm* dst, const PointXfrm* src);
void identity_PointXfrm (PointXfrm* xfrm);
void rotation_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle);
void col_PointXfrm (Point* dst, const PointXfrm* xfrm, uint col);
void xfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src);
void xfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B);
void trxfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src);
void reflect_PointXfrm (PointXfrm* xfrm, uint j, uint k);
void to_basis_PointXfrm (PointXfrm* dst, const PointXfrm* xfrm,
                         const PointXfrm* basis);
void orthonormalize_PointXfrm (PointXfrm* dst, const PointXfrm* A);
void orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A, uint dim);

#include "xfrm.c"
#endif

