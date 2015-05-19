
#include "serial.h"
#include "affine.h"
#include "xfrm.h"

  void
oput_Point (OFile* of, const Point* x)
{
  oput_char_OFile (of, '(');
  {:for (ci ; NDims)
    if (ci > 0)  oput_char_OFile (of, ',');
    oput_real_OFile (of, x->coords[ci]);
  }
  oput_char_OFile (of, ')');
}

  void
oput_PointXfrm (OFile* of, const PointXfrm* A)
{
  oput_char_OFile (of, '[');
  {:for (ci ; NDims)
    if (ci > 0)  oput_char_OFile (of, ',');
    oput_Point (of, &A->pts[ci]);
  }
  oput_char_OFile (of, ']');
}

  void
oput_IAMap (OFile* of, const IAMap* A)
{
  oput_char_OFile (of, '{');
  oput_Point (of, &A->xlat);
  oput_Point (of, &A->scale);
  oput_PointXfrm (of, &A->xfrm);
  oput_char_OFile (of, '}');
}

  bool
xget_Point (XFile* xf, Point* x)
{
  nextds_XFile (xf, NULL, "(");
  {:for (i ; NDims)
    if (!xget_real_XFile (xf, &x->coords[i]))
      return false;
    nextds_XFile (xf, NULL, (i < NDims-1) ? "," : ")");
  }
  return true;
}

  bool
xget_PointXfrm (XFile* xf, PointXfrm* A)
{
  nextds_XFile (xf, NULL, "[");
  {:for (i ; NDims)
    if (!xget_Point (xf, &A->pts[i]))
      return false;
    nextds_XFile (xf, NULL, (i < NDims-1) ? "," : "]");
  }
  return true;
}

  bool
xget_IAMap (XFile* xf, IAMap* A)
{
  nextds_XFile (xf, NULL, "{");
  if (!xget_Point (xf, &A->xlat))  return false;
  if (!xget_Point (xf, &A->scale))  return false;
  if (!xget_PointXfrm (xf, &A->xfrm))  return false;
  nextds_XFile (xf, NULL, "}");
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

