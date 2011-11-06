
#ifndef XFRM_H_
#ifndef __OPENCL_VERSION__
#define XFRM_H_
#include "space.h"
#endif  /* #ifndef __OPENCL_VERSION__ */

    /* #define ForwardDim (NDimensions - 1) */
#define UpDim 0
#define RightDim 1
#define ForwardDim 2

typedef struct PointXfrm PointXfrm;

struct PointXfrm
{
    Point pts[NDimensions];
};

void zero_PointXfrm (PointXfrm* xfrm);
void copy_PointXfrm (PointXfrm* dst, const PointXfrm* src);
void identity_PointXfrm (PointXfrm* xfrm);
void
scale_PointXfrm (PointXfrm* dst, const PointXfrm* src, real a);
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
void
orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A,
                       const Point* v_in, uint dim);
void
stable_orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A,
                              const Point* v_in, uint dim);

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
              const Point* old_centroid);
void
point_from_basis (Point* ret_origin,
                const PointXfrm* basis,
                const Point* origin,
                const Point* new_centroid);
void
ray_from_basis (Point* ret_origin, Point* ret_dir,
                const PointXfrm* basis,
                const Point* origin,
                const Point* dir,
                const Point* new_centroid);

#ifndef __OPENCL_VERSION__
void output_PointXfrm (FILE* out, const PointXfrm* xfrm);
#ifdef INCLUDE_SOURCE
#include "xfrm.c"
#endif
#endif  /* #ifndef __OPENCL_VERSION__ */

#endif

