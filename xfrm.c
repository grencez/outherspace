
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

void zero_PointXfrm (PointXfrm* xfrm)
{
    memset (xfrm, 0, sizeof (PointXfrm));
}

void copy_PointXfrm (PointXfrm* dst, const PointXfrm* src)
{
    memcpy (dst, src, sizeof (PointXfrm));
}

void identity_PointXfrm (PointXfrm* xfrm)
{
    uint i;
    zero_PointXfrm (xfrm);
    UFor( i, NDimensions )
        xfrm->pts[i].coords[i] = 1;
}

void rotation_PointXfrm (PointXfrm* xfrm, uint xdim, uint ydim, real angle)
{
    real tcos, tsin;
    identity_PointXfrm (xfrm);

    tcos = cos (angle);
    tsin = sin (angle);

    xfrm->pts[xdim].coords[xdim] =   tcos;
    xfrm->pts[xdim].coords[ydim] = - tsin;
    xfrm->pts[ydim].coords[ydim] =   tcos;
    xfrm->pts[ydim].coords[xdim] =   tsin;
}

void col_PointXfrm (Point* dst, const PointXfrm* xfrm, uint col)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = xfrm->pts[i].coords[col];
}

void xfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src)
{
    uint i;
    UFor( i, NDimensions )
        dst->coords[i] = dot_Point (src, &xfrm->pts[i]);
}

void trxfrm_Point (Point* dst, const PointXfrm* xfrm, const Point* src)
{
    uint i;
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

void xfrm_PointXfrm (PointXfrm* dst, const PointXfrm* A, const PointXfrm* B)
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

void orthonormalize_PointXfrm (PointXfrm* dst, const PointXfrm* A)
{
    uint i;
    assert (dst != A);
    UFor( i, NDimensions )
    {
        uint j;
        Point tmp;
        copy_Point (&tmp, &A->pts[i]);
        UFor( j, i )
        {
            Point proj;
            proj_Point (&proj, &A->pts[i], &A->pts[j]);
            diff_Point (&tmp, &tmp, &proj);
        }
        normalize_Point (&dst->pts[i], &tmp);
    }
}

