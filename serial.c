
#include "serial.h"
#include "affine.h"
#include "xfrm.h"

    void
dumpp_Point (FileB* f, const Point* x)
{
    dump_char_FileB (f, '(');
    { BLoop( ci, NDims )
        if (ci > 0)  dump_char_FileB (f, ',');
        dump_real_FileB (f, x->coords[ci]);
    } BLose()
    dump_char_FileB (f, ')');
}

    void
dumpp_PointXfrm (FileB* f, const PointXfrm* A)
{
    dump_char_FileB (f, '[');
    { BLoop( ci, NDims )
        if (ci > 0)  dump_char_FileB (f, ',');
        dumpp_Point (f, &A->pts[ci]);
    } BLose()
    dump_char_FileB (f, ']');
}

    void
dumpp_IAMap (FileB* f, const IAMap* A)
{
    dump_char_FileB (f, '{');
    dumpp_Point (f, &A->xlat);
    dumpp_Point (f, &A->scale);
    dumpp_PointXfrm (f, &A->xfrm);
    dump_char_FileB (f, '}');
}

    bool
load_Point (FileB* f, Point* x)
{
    if (!f->good)  return false;
    nextds_FileB (f, NULL, "(");
    { BLoop( i, NDims )
        load_real_FileB (f, &x->coords[i]);
        nextds_FileB (f, NULL, (i < NDims-1) ? "," : ")");
    } BLose()
    return f->good;
}

    bool
load_PointXfrm (FileB* f, PointXfrm* A)
{
    if (!f->good)  return false;
    nextds_FileB (f, NULL, "[");
    { BLoop( i, NDims )
        load_Point (f, &A->pts[i]);
        nextds_FileB (f, NULL, (i < NDims-1) ? "," : "]");
    } BLose()
    return f->good;
}

    bool
load_IAMap (FileB* f, IAMap* A)
{
    nextds_FileB (f, NULL, "{");
    load_Point (f, &A->xlat);
    load_Point (f, &A->scale);
    load_PointXfrm (f, &A->xfrm);
    nextds_FileB (f, NULL, "}");
        /* normalize_IAMap (A); */
    return f->good;
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

