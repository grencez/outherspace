
#include "serial.h"
#include "affine.h"
#include "xfrm.h"

    void
oput_Point (OFileB* of, const Point* x)
{
    oput_char_OFileB (of, '(');
    {:for (ci ; NDims)
        if (ci > 0)  oput_char_OFileB (of, ',');
        oput_real_OFileB (of, x->coords[ci]);
    }
    oput_char_OFileB (of, ')');
}

    void
oput_PointXfrm (OFileB* of, const PointXfrm* A)
{
    oput_char_OFileB (of, '[');
    {:for (ci ; NDims)
        if (ci > 0)  oput_char_OFileB (of, ',');
        oput_Point (of, &A->pts[ci]);
    }
    oput_char_OFileB (of, ']');
}

    void
oput_IAMap (OFileB* of, const IAMap* A)
{
    oput_char_OFileB (of, '{');
    oput_Point (of, &A->xlat);
    oput_Point (of, &A->scale);
    oput_PointXfrm (of, &A->xfrm);
    oput_char_OFileB (of, '}');
}

    bool
xget_Point (XFileB* xf, Point* x)
{
    nextds_XFileB (xf, NULL, "(");
    {:for (i ; NDims)
        if (!xget_real_XFileB (xf, &x->coords[i]))
            return false;
        nextds_XFileB (xf, NULL, (i < NDims-1) ? "," : ")");
    }
    return true;
}

    bool
xget_PointXfrm (XFileB* xf, PointXfrm* A)
{
    nextds_XFileB (xf, NULL, "[");
    {:for (i ; NDims)
        if (!xget_Point (xf, &A->pts[i]))
            return false;
        nextds_XFileB (xf, NULL, (i < NDims-1) ? "," : "]");
    }
    return true;
}

    bool
xget_IAMap (XFileB* xf, IAMap* A)
{
    nextds_XFileB (xf, NULL, "{");
    if (!xget_Point (xf, &A->xlat))  return false;
    if (!xget_Point (xf, &A->scale))  return false;
    if (!xget_PointXfrm (xf, &A->xfrm))  return false;
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

