
#ifndef __OPENCL_VERSION__
#include "xfrm.h"

#include <assert.h>
#include <math.h>
#include <string.h>

void output_PointXfrm (FILE* out, const PointXfrm* xfrm)
{
    uint pi;
    const char* delim = "";
    fputc ('[', out);
    UFor( pi, NDimensions )
    {
        fputs (delim, out);
        output_Point (out, &xfrm->pts[pi]);
        delim = " ";
    }
    fputc (']', out);
}
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
#ifdef __OPENCL_VERSION__
    uint i;
    UFor( i, NDimensions )
        copy_Point (&dst->pts[i], &src->pts[i]);
#else
    memcpy (dst, src, sizeof (PointXfrm));
#endif
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

void rotation_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle)
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
    uint i;
    assert (dst != src);
    UFor( i, NDimensions )
        dst->coords[i] = dot_Point (src, &xfrm->pts[i]);
}

    void
trxfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src)
{
    uint i;
    assert (dst != src);
    zero_Point (dst);
    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, NDimensions )
        {
            dst->coords[i] += src->coords[j] * xfrm->pts[j].coords[i];
        }
    }
}

    void
xfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
{
    uint j;
    assert (dst != A);
    assert (dst != B);

    UFor( j, NDimensions )
    {
        uint i;
        Point B_col, dst_col;
        col_PointXfrm (&B_col, B, j);

        xfrm_Point (&dst_col, A, &B_col);

        UFor( i, NDimensions )
            dst->pts[i].coords[j] = dst_col.coords[i];
    }
}

    void
trxfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
{
    uint j;
    assert (dst != A);
    assert (dst != B);

    UFor( j, NDimensions )
    {
        uint i;
        Point B_col, dst_col;
        col_PointXfrm (&B_col, B, j);

        trxfrm_Point (&dst_col, A, &B_col);

        UFor( i, NDimensions )
            dst->pts[i].coords[j] = dst_col.coords[i];
    }
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

void reflect_PointXfrm (PointXfrm* xfrm, uint j, uint k)
{
    Point rows[2];
    uint i;

    copy_Point (&rows[0], &xfrm->pts[j]);
    copy_Point (&rows[1], &xfrm->pts[k]);

        /* Swap indices /j/ and /k/.*/
    UFor( i, 2 )
    {
        real tmp;
        tmp = rows[i].coords[j];
        rows[i].coords[j] = rows[i].coords[k];
        rows[i].coords[k] = tmp;
    }

    copy_Point (&xfrm->pts[j], &rows[1]);
    copy_Point (&xfrm->pts[k], &rows[0]);
}

void swaprows_PointXfrm (PointXfrm* xfrm, uint j, uint k)
{
    Point tmp;
    copy_Point (&tmp, &xfrm->pts[j]);
    copy_Point (&xfrm->pts[j], &xfrm->pts[k]);
    copy_Point (&xfrm->pts[k], &tmp);
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
        Point tmp;
        copy_Point (&tmp, &A->pts[i]);
        UFor( j, i )
        {
            Point proj;
            proj_Point (&proj, &tmp, &dst->pts[j]);
            diff_Point (&tmp, &tmp, &proj);
        }
        normalize_Point (&dst->pts[i], &tmp);
    }
}

void orthorotate_PointXfrm (PointXfrm* dst, const PointXfrm* A, uint dim)
{
    uint i = dim;
    assert (dst != A);

        /* Perform Gram-Schmidt process, but starting from a specific
         * dimension so it can be adjusted to achieve rotation.
         */
    normalize_Point (&dst->pts[i], &A->pts[i]);
    while (i != 0)
    {
        uint j;
        Point tmp;
        --i;
        copy_Point (&tmp, &A->pts[i]);

        for (j = dim; j != i; --j)
        {
            Point proj;
            proj_Point (&proj, &tmp, &dst->pts[j]);
            diff_Point (&tmp, &tmp, &proj);
        }
        normalize_Point (&dst->pts[i], &tmp);
    }

    for (i = dim+1; i < NDimensions; ++i)
    {
        uint j;
        Point tmp;
        copy_Point (&tmp, &A->pts[i]);
        UFor( j, i )
        {
            Point proj;
            proj_Point (&proj, &tmp, &dst->pts[j]);
            diff_Point (&tmp, &tmp, &proj);
        }
        normalize_Point (&dst->pts[i], &tmp);
    }
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

/* Take a bounding box, rotate it from a basis,
 * and reposition it around a new centroid.
 * The bounding box volume will grow in most cases,
 * as it is axis aligned and the axes have changed!
 */
    void
trxfrm_BoundingBox (BoundingBox* dst,
                    const PointXfrm* basis,
                    const BoundingBox* box,
                    const Point* new_centroid)
{
    uint i;
    Point diff;
    PointXfrm bbox, robox;

    Op_Point_1200( &diff ,.5*, -, &box->max , &box->min );

    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, NDimensions )
        {
            if (basis->pts[j].coords[i] >= 0)
                bbox.pts[i].coords[j] = + diff.coords[j];
            else
                bbox.pts[i].coords[j] = - diff.coords[j];
        }
    }

        /*    T   T T
         *  (B * A )  = A * B
         */
    xfrm_PointXfrm (&robox, &bbox, basis);

    UFor( i, NDimensions )
    {
        uint j;
        real a, b;
        a = robox.pts[i].coords[i];
        b = new_centroid->coords[i];
        dst->min.coords[i] = - a + b;
        dst->max.coords[i] = + a + b;

        UFor( j, NDimensions )
            assert (a >= robox.pts[j].coords[i]);
    }
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

    copy_Point (&diff, dir);
    xfrm_Point (ret_dir, basis, &diff);
}

    void
ray_from_basis (Point* ret_origin, Point* ret_dir,
                const PointXfrm* basis,
                const Point* origin,
                const Point* dir,
                const Point* new_centroid)
{
    Point diff;

    trxfrm_Point (&diff, basis, origin);
    summ_Point (ret_origin, &diff, new_centroid);

    copy_Point (&diff, dir);
    trxfrm_Point (ret_dir, basis, &diff);
}

