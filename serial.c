
#include "serial.h"
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
    dump_char_FileB (f, ')');
}



void output_Point (FILE* out, const Point* point)
{
    uint ci;
    const char* delim = "";
    fputc ('(', out);
    UFor( ci, NDimensions )
    {
            /* fprintf (out, "%s%4.1f", delim, point->coords[ci]); */
        fprintf (out, "%s%.2f", delim, point->coords[ci]);
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

