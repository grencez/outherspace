
#ifndef __OPENCL_VERSION__
#include "xfrm.h"

#include "point.h"
#include "util.h"

#include <assert.h>
#include <math.h>
#include <string.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

void zero_PointXfrm (PointXfrm* xfrm)
{
#ifdef __OPENCL_VERSION__
    uint i;
    UFor( i, NDimensions )
        zero_Point (&xfrm->pts[i]);
#else
    memset (xfrm, 0, sizeof (PointXfrm));
#endif
}

void copy_PointXfrm (PointXfrm* dst, const PointXfrm* src)
{
    *dst = *src;
}

void identity_PointXfrm (PointXfrm* xfrm)
{
    uint i;
    zero_PointXfrm (xfrm);
    UFor( i, NDimensions )
        xfrm->pts[i].coords[i] = 1;
}

    void
scale_PointXfrm (PointXfrm* dst, const PointXfrm* src, real a)
{
    uint i;
    UFor( i, NDimensions )
        scale_Point (&dst->pts[i], &src->pts[i], a);
}

    /** Get a rotation matrix for /t/ revolutions.**/
    void
rotn_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real t)
{
    rotation_PointXfrm (xfrm, xdim, ydim, 2 * M_PI * t);
}

    void
rotation_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle)
{
    real tcos, tsin;
    identity_PointXfrm (xfrm);

    tcos = cos (angle);
    tsin = sin (angle);

    xfrm->pts[xdim].coords[xdim] =   tcos;
    xfrm->pts[xdim].coords[ydim] = - tsin;
    xfrm->pts[ydim].coords[xdim] =   tsin;
    xfrm->pts[ydim].coords[ydim] =   tcos;
}

    /** Rotate /xdim/ in the direction of /ydim/ by /angle/.
     * This is the same as pre-multiplying by a matrix representing
     * an /angle/ rotation about the (/xdim/,/ydim/)-plane.
     **/
    void
rotate_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle)
{
    uint i;
    real tcos, tsin;
    Point* u;
    Point* v;

    tcos = cos (angle);
    tsin = sin (angle);

    u = &xfrm->pts[xdim];
    v = &xfrm->pts[ydim];

    UFor( i, NDimensions )
    {
        real x, y;
        x = u->coords[i];
        y = v->coords[i];
        u->coords[i] = tcos * x - tsin * y;
        v->coords[i] = tsin * x + tcos * y;
    }
}

    void
trrotate_PointXfrm (PointXfrm* basis, uint xdim, uint ydim, real angle)
{
    rotate_PointXfrm (basis, xdim, ydim, -angle);
}

    void
rotatetr_PointXfrm (PointXfrm* basis, uint xdim, uint ydim, real angle)
{
    uint i;
    real tcos, tsin;

    tcos = cos (angle);
    tsin = sin (angle);

    UFor( i, NDimensions )
    {
        real x, y;
        x = basis->pts[i].coords[xdim];
        y = basis->pts[i].coords[ydim];
        basis->pts[i].coords[xdim] = tcos * x - tsin * y;
        basis->pts[i].coords[ydim] = tsin * x + tcos * y;
    }
}

void col_PointXfrm (Point* dst, const PointXfrm* xfrm, uint col)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = xfrm->pts[i].coords[col];
}

    void
xfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src)
{
    Point u;
    uint i;
    UFor( i, NDimensions )
        u.coords[i] = dot_Point (src, &xfrm->pts[i]);
    *dst = u;
}

    void
trxfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src)
{
    Point u;
    uint i;
    zero_Point (&u);
    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, NDimensions )
            u.coords[i] += src->coords[j] * xfrm->pts[j].coords[i];
    }
    *dst = u;
}

    void
xfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
{
    uint j;
    PointXfrm C;

    UFor( j, NDimensions )
    {
        Point B_col;
        col_PointXfrm (&B_col, B, j);
        xfrm_Point (&C.pts[j], A, &B_col);
    }
    transpose_PointXfrm (dst, &C);
}

    void
trxfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
{
    uint j;
    PointXfrm C;

    UFor( j, NDimensions )
    {
        Point B_col;
        col_PointXfrm (&B_col, B, j);
        trxfrm_Point (&C.pts[j], A, &B_col);
    }
    transpose_PointXfrm (dst, &C);
}

    void
xfrmtr_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
{
    uint i;
    assert (dst != A);
    assert (dst != B);

    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, NDimensions )
            dst->pts[i].coords[j] = dot_Point (&A->pts[i], &B->pts[j]);
    }
}

    /** Create a matrix that maps vectors to a different permutation of its
     * coordinates. Odd values in the permutation array mark negation.
     * The identity matrix is formed by /perms[i]=2*i/.
     *
     * The equation for applying the matrix to a vector /x/ is
     *   /x'[i] = x[perms[i]/2] * (-1)^perms[i]/
     **/
    void
permutation_PointXfrm (PointXfrm* a, const uint* perms)
{
    uint i;
    zero_PointXfrm (a);
    UFor( i, NDimensions )
        a->pts[i].coords[perms[i]/2] = even_uint (perms[i]) ? 1 : -1;
}

void reflect_PointXfrm (PointXfrm* xfrm, uint j, uint k)
{
    Point rows[2];
    uint i;

    rows[0] = xfrm->pts[j];
    rows[1] = xfrm->pts[k];

        /* Swap indices /j/ and /k/.*/
    UFor( i, 2 )
    {
        real tmp;
        tmp = rows[i].coords[j];
        rows[i].coords[j] = rows[i].coords[k];
        rows[i].coords[k] = tmp;
    }

    xfrm->pts[j] = rows[1];
    xfrm->pts[k] = rows[0];
}

void swaprows_PointXfrm (PointXfrm* xfrm, uint j, uint k)
{
    Point tmp;
    tmp = xfrm->pts[j];
    xfrm->pts[j] = xfrm->pts[k];
    xfrm->pts[k] = tmp;
}

void to_basis_PointXfrm (PointXfrm* dst, const PointXfrm* xfrm,
                         const PointXfrm* basis)
{
    uint j;
    assert (dst != xfrm);
    assert (dst != basis);

    UFor( j, NDimensions )
    {
        Point basis_col, tb_col;
        col_PointXfrm (&basis_col, basis, j);

        xfrm_Point (&tb_col, xfrm, &basis_col);
        trxfrm_Point (&dst->pts[j], basis, &tb_col);
    }
}

void transpose_PointXfrm (PointXfrm* dst, const PointXfrm* xfrm)
{
    uint i;
    UFor( i, NDimensions )
    {
        uint j;
        dst->pts[i].coords[i] = xfrm->pts[i].coords[i];
        UFor( j, i )
        {
            real x, y;
            x = xfrm->pts[i].coords[j];
            y = xfrm->pts[j].coords[i];
            dst->pts[i].coords[j] = y;
            dst->pts[j].coords[i] = x;
        }
    }
}

void orthonormalize_PointXfrm (PointXfrm* dst, const PointXfrm* A)
{
    uint i;
    assert (dst != A);
        /* Perform (numerically stable) Gram-Schmidt process.*/
    UFor( i, NDimensions )
    {
        uint j;
        Point tmp = A->pts[i];
        UFor( j, i )
        {
            Point proj;
            proj_Point (&proj, &tmp, &dst->pts[j]);
            diff_Point (&tmp, &tmp, &proj);
        }
        normalize_Point (&dst->pts[i], &tmp);
    }
}

    /** Rotate an orthonormal basis given
     * a new vector /v_in/ to use for the /dim/th basis vector.
     * The current basis vectors are assumed to be rows in /A/.
     *
     * /dst/ can be the same as /A/.
     * /v_in/ need not be normalized.
     **/
    void
orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A,
                       const Point* v_in, uint dim)
{
    Point u, v, w;
    uint i;
    real d;

    normalize_Point (&v, v_in);
    u = A->pts[dim];
    d = 1 + dot_Point (&u, &v);
    summ_Point (&w, &u, &v);

    UFor( i, NDimensions )
    {
        Point x;
        real c;

        x = A->pts[i];
        c = dot_Point (&v, &x) / d;

        Op_Point_2010( &dst->pts[i] ,-, &x ,c*, &w );
    }

    dst->pts[dim] = v;
}

    /** Much like orthorotate_PointXfrm() but
     * handles half-rotations in a more stable manner
     * by doing two quarter-rotations.
     * This bounds the maximal value used in the calculation
     * (aside from normalization) to 2.
     **/
    void
stable_orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A,
                              const Point* v_in, uint dim)
{
    Point v;

    if (0 <= dot_Point (&A->pts[dim], v_in))
    {
        orthorotate_PointXfrm (dst, A, v_in, dim);
        return;
    }

    orth_unit_Point (&v, v_in, &A->pts[dim]);
    if (Epsilon_real >= dot_Point (&v, &v))
    {
        if (dim == 0)  v = A->pts[1];
        else           v = A->pts[0];
    }

    orthorotate_PointXfrm (dst, A, &v, dim);
    orthorotate_PointXfrm (dst, dst, v_in, dim);
}

real det2_PointXfrm (const PointXfrm* xfrm,
                     uint ri, uint rj, uint ci, uint cj)
{
    return (xfrm->pts[ri].coords[ci] * xfrm->pts[rj].coords[cj] -
            xfrm->pts[ri].coords[cj] * xfrm->pts[rj].coords[ci]);
}

real det3_PointXfrm (const PointXfrm* xfrm,
                     uint ri, uint rj, uint rk,
                     uint ci, uint cj, uint ck)
{
    real a[3][3];
    a[0][0] = xfrm->pts[ri].coords[ci];
    a[0][1] = xfrm->pts[ri].coords[cj];
    a[0][2] = xfrm->pts[ri].coords[ck];
    a[1][0] = xfrm->pts[rj].coords[ci];
    a[1][1] = xfrm->pts[rj].coords[cj];
    a[1][2] = xfrm->pts[rj].coords[ck];
    a[2][0] = xfrm->pts[rk].coords[ci];
    a[2][1] = xfrm->pts[rk].coords[cj];
    a[2][2] = xfrm->pts[rk].coords[ck];
    return (+ a[0][0]*a[1][1]*a[2][2]
            + a[0][1]*a[1][2]*a[2][0]
            + a[0][2]*a[1][0]*a[2][1]
            - a[2][0]*a[1][1]*a[0][2]
            - a[2][1]*a[1][2]*a[0][0]
            - a[2][2]*a[1][0]*a[0][1]);
}

real det_PointXfrm (const PointXfrm* xfrm)
{
    Point p;
    row_minors_PointXfrm (&p, xfrm, 0);
    checker_negate_Point (&p);
    return  dot_Point (&p, &xfrm->pts[0]);
}

void row_minors_PointXfrm (Point* dst, const PointXfrm* xfrm, uint row)
{
#if NDimensions == 3
    uint i, j;
    if      (row == 0) { i = 1; j = 2; }
    else if (row == 1) { i = 0; j = 2; }
    else               { i = 0; j = 1; }

    dst->coords[0] = det2_PointXfrm (xfrm, i, j, 1, 2);
    dst->coords[1] = det2_PointXfrm (xfrm, i, j, 0, 2);
    dst->coords[2] = det2_PointXfrm (xfrm, i, j, 0, 1);
#elif NDimensions == 4
    uint i, j, k;
    if      (row == 0) { i = 1; j = 2; k = 3; }
    else if (row == 1) { i = 0; j = 2; k = 3; }
    else if (row == 2) { i = 0; j = 1; k = 3; }
    else               { i = 0; j = 1; k = 2; }

    dst->coords[0] = det3_PointXfrm (xfrm, i, j, k, 1, 2, 3);
    dst->coords[1] = det3_PointXfrm (xfrm, i, j, k, 0, 2, 3);
    dst->coords[2] = det3_PointXfrm (xfrm, i, j, k, 0, 1, 3);
    dst->coords[3] = det3_PointXfrm (xfrm, i, j, k, 0, 1, 2);
#else
    assert (0 && "Not implemented.");
#endif
}

    void
spherical3_PointXfrm (PointXfrm* dst, real zenith, real azimuthcc)
{
    identity_PointXfrm (dst);
    rotate_PointXfrm (dst, RightDim, ForwardDim, azimuthcc);
    rotate_PointXfrm (dst, UpDim,    ForwardDim, M_PI / 2 - zenith);
}

    void
ray_to_basis (Point* ret_origin, Point* ret_dir,
              const PointXfrm* basis,
              const Point* origin,
              const Point* dir,
              const Point* old_centroid)
{
    Point diff;

    diff_Point (&diff, origin, old_centroid);
    xfrm_Point (ret_origin, basis, &diff);

    diff = *dir;
    xfrm_Point (ret_dir, basis, &diff);
}

    void
point_from_basis (Point* ret_origin,
                const PointXfrm* basis,
                const Point* origin,
                const Point* new_centroid)
{
    Point diff;
    trxfrm_Point (&diff, basis, origin);
    summ_Point (ret_origin, &diff, new_centroid);
}

    void
ray_from_basis (Point* ret_origin, Point* ret_dir,
                const PointXfrm* basis,
                const Point* origin,
                const Point* dir,
                const Point* new_centroid)
{
    Point diff;
    point_from_basis (ret_origin, basis, origin, new_centroid);

    diff = *dir;
    trxfrm_Point (ret_dir, basis, &diff);
}

