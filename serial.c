
#include "serial.h"
#include "affine.h"
#include "xfrm.h"

    void
dumpp_Point (OFileB* of, const Point* x)
{
    dump_char_OFileB (of, '(');
    { BLoop( ci, NDims )
        if (ci > 0)  dump_char_OFileB (of, ',');
        dump_real_OFileB (of, x->coords[ci]);
    } BLose()
    dump_char_OFileB (of, ')');
}

    void
dumpp_PointXfrm (OFileB* of, const PointXfrm* A)
{
    dump_char_OFileB (of, '[');
    { BLoop( ci, NDims )
        if (ci > 0)  dump_char_OFileB (of, ',');
        dumpp_Point (of, &A->pts[ci]);
    } BLose()
    dump_char_OFileB (of, ']');
}

    void
dumpp_IAMap (OFileB* of, const IAMap* A)
{
    dump_char_OFileB (of, '{');
    dumpp_Point (of, &A->xlat);
    dumpp_Point (of, &A->scale);
    dumpp_PointXfrm (of, &A->xfrm);
    dump_char_OFileB (of, '}');
}

    bool
load_Point (XFileB* xf, Point* x)
{
    nextds_XFileB (xf, NULL, "(");
    { BLoop( i, NDims )
        if (!load_real_XFileB (xf, &x->coords[i]))
            return false;
        nextds_XFileB (xf, NULL, (i < NDims-1) ? "," : ")");
    } BLose()
    return true;
}

    bool
load_PointXfrm (XFileB* xf, PointXfrm* A)
{
    nextds_XFileB (xf, NULL, "[");
    { BLoop( i, NDims )
        if (!load_Point (xf, &A->pts[i]))
            return false;
        nextds_XFileB (xf, NULL, (i < NDims-1) ? "," : "]");
    } BLose()
    return true;
}

    bool
load_IAMap (XFileB* xf, IAMap* A)
{
    nextds_XFileB (xf, NULL, "{");
    if (!load_Point (xf, &A->xlat))  return false;
    if (!load_Point (xf, &A->scale))  return false;
    if (!load_PointXfrm (xf, &A->xfrm))  return false;
    nextds_XFileB (xf, NULL, "}");
        /* normalize_IAMap (A); */
    return true;
}

void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
            /* fprintf (out, "%s%4.1f", delim, point->coords[ci]); */
        fprintf (out, "%s%.5f", delim, point->coords[ci]);
            /* delim = ", "; */
        delim = " ";
    }
    fputc (')', out);
}

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

void output_Simplex (FILE* out, const Simplex* elem)
{
    uint pi;
    const char* delim = "";
    fputc ('[', out);
    UFor( pi, NDimensions )
    {
        fputs (delim, out);
        output_Point (out, &elem->pts[pi]);
        delim = " ";
    }
    fputc (']', out);
}

void output_BBox (FILE* out, const BBox* box)
{
    fputs ("BBox: ", out);
    output_Point (out, &box->min);
    fputs (" to ", out);
    output_Point (out, &box->max);
}

