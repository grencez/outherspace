
#ifndef XFRM_H_
#ifndef __OPENCL_VERSION__
#define XFRM_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

    /* #define DirDimension (NDimensions - 1) */
#define DirDimension 2

struct point_xfrm_struct
{
    Point pts[NDimensions];
};
typedef struct point_xfrm_struct PointXfrm;

void zero_PointXfrm (PointXfrm* xfrm);
void copy_PointXfrm (PointXfrm* dst, const PointXfrm* src);
void identity_PointXfrm (PointXfrm* xfrm);
void rotation_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle);
void
rotate_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle);
void
trrotate_PointXfrm (PointXfrm* basis, uint xdim, uint ydim, real angle);
void
rotatetr_PointXfrm (PointXfrm* basis, uint xdim, uint ydim, real angle);
void col_PointXfrm (Point* dst, const PointXfrm* xfrm, uint col);
void
xfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src);
void
trxfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src);
void
xfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B);
void
trxfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B);
void
xfrmtr_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B);
void reflect_PointXfrm (PointXfrm* xfrm, uint j, uint k);
void swaprows_PointXfrm (PointXfrm* xfrm, uint j, uint k);
void to_basis_PointXfrm (PointXfrm* dst, const PointXfrm* xfrm,
                         const PointXfrm* basis);
void transpose_PointXfrm (PointXfrm* dst, const PointXfrm* xfrm);
void orthonormalize_PointXfrm (PointXfrm* dst, const PointXfrm* A);
void orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A, uint dim);

real det2_PointXfrm (const PointXfrm* xfrm,
                     uint ri, uint rj, uint ci, uint cj);
real det3_PointXfrm (const PointXfrm* xfrm,
                     uint ri, uint rj, uint rk,
                     uint ci, uint cj, uint ck);
real
det_PointXfrm (const PointXfrm* xfrm);
void row_minors_PointXfrm (Point* dst, const PointXfrm* xfrm, uint row);
void
spherical3_PointXfrm (PointXfrm* dst, real zenith, real azimuthcc);
void
trxfrm_BoundingBox (BoundingBox* dst,
                    const PointXfrm* basis,
                    const BoundingBox* box,
                    const Point* new_centroid);
void
ray_to_basis (Point* ret_origin, Point* ret_dir,
              const PointXfrm* basis,
              const Point* origin,
              const Point* dir,
              const Point* old_centroid,
              const BoundingBox* box);

#ifndef __OPENCL_VERSION__
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
#ifdef INCLUDE_SOURCE
#include "xfrm.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

