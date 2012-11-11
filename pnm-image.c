
#include "cx/fileb.h"
#include "pnm-image.h"
#include "space.h"

#include <string.h>


void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    uint row;
    FILE* out;

    out = fopen (filename, "wb");
    fputs ("P1\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);

    UFor( row, nrows )
    {
        uint col;
        const uint* hitline;

        hitline = &hits[(nrows - row - 1) * ncols];
        UFor( col, ncols )
        {
            if (hitline[col] < nelems)
                fputs (" 1", out);
            else
                fputs (" 0", out);
        }
        fputc ('\n', out);
    }
    fclose (out);
}


void output_PGM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    const bool debug = false;
    uint img_row;
    FILE* out;

    out = fopen (filename, "wb");
    if (!out)
    {
        fprintf (stderr, "Cannot open file for writing:%s\n", filename);
        return;
    }

    fputs ("P2\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);
    fprintf (out, "%u\n", nelems);

    if (debug)
        fprintf (out, "# nelems:%u  maxval:%u  (elem_idx = val - 1)\n",
                 nelems, nelems+1);

    UFor( img_row, nrows )
    {
        uint row, col;
        const uint* hitline;

        row = nrows - img_row - 1;
        hitline = &hits[row * ncols];
        if (debug)  fprintf (out, "# row:%u\n", row);

        UFor( col, ncols )
        {
            uint v;
            v = (hitline[col] < nelems) ? hitline[col] : nelems;
            v += 1;
            fprintf (out, " %u", v);
        }
        fputc ('\n', out);
    }
    fclose (out);
}


void output_PPM_image (const char* filename, uint nrows, uint ncols,
                       const byte* pixels)
{
    FileB ofb = dflt_FileB ();
    OFileB* of = &ofb.xo;

    ofb.sink = true;
    if (!open_FileB (&ofb, 0, filename))
    {
        fprintf (stderr, "Cannot open file for writing:%s\n", filename);
        return;
    }

        /* oput_cstr_FileB (f, "P3\n"); */
    oput_cstr_OFileB (of, "P6\n");
    oput_uint_OFileB (of, ncols);
    oput_char_OFileB (of, ' ');
    oput_uint_OFileB (of, nrows);
    oput_char_OFileB (of, '\n');
    oput_cstr_OFileB (of, "255\n");

#if 0
    {:for (row ; nrows)
        const byte* pixline;
        pixline = &pixels[(nrows - row - 1) * 3 * ncols];
        {:for (col ; ncols)
            oput_char_OFileB (of, ' ');
            oput_uint_OFileB (of, pixline[3*col+0]);
            oput_char_OFileB (of, ' ');
            oput_uint_OFileB (of, pixline[3*col+1]);
            oput_char_OFileB (of, ' ');
            oput_uint_OFileB (of, pixline[3*col+2]);
        }
        oput_char_OFileB (of, '\n');
    }
#else
    setfmt_FileB (&ofb, FileB_Raw);
    {:for (row ; nrows)
        oputn_byte_FileB (&ofb,
                          &pixels[(nrows - row - 1) * NColors * ncols],
                          ncols * NColors);
    }
#endif

    lose_FileB (&ofb);
}


    byte*
readin_PPM_image (uint* ret_nrows, uint* ret_ncols,
                  const char* pathname, const char* filename)
{
    uint nrows = 0, ncols = 0;
    bool good = true;
    byte* pixels;
    FileB xfb = dflt_FileB ();
    XFileB* xf = &xfb.xo;
    uint max_color_value = 255;
    uint header_stage;
    bool ascii = true;
    real t0;
    const char* line;

    t0 = monotime ();

    if (!open_FileB (&xfb, pathname, filename))
    {
        fprintf (stderr, "Cannot open file for reading:%s/%s\n",
                 pathname, filename);
        return 0;
    }


    header_stage = 0;
    while (good && header_stage < 3 && (line = getline_XFileB (xf)))
    {
        DecloStack1( XFileB, olay, olay_XFileB (xf, IdxEltTable( xf->buf, line )) );
        skipds_XFileB (olay, 0);
        line = cstr_XFileB (olay);

        if (line[0] == '#')
        {
                /* Do nothing!*/
        }
        else if (header_stage == 0)
        {
            header_stage += 1;
            if (0 == strcmp (line, "P6"))
                ascii = false;
            else
                good = (0 == strcmp (line, "P3"));

            if (!good)  fprintf (stderr, "Invalid PPM type:%s\n", line);
        }
        else if (header_stage == 1)
        {
            header_stage += 1;
            line = strto_uint (&ncols, line);
            if (line)
                line = strto_uint (&nrows, line);

            good = (line != 0);
        }
        else if (header_stage == 2)
        {
            header_stage += 1;
            line = strto_uint (&max_color_value, line);
            good = (line != 0);
        }
    }

    if (!good)
    {
        lose_FileB (&xfb);
        return 0;
    }

    if (!ascii)  setfmt_FileB (&xfb, FileB_Raw);

    pixels = AllocT( byte, nrows * ncols * NColors );

    {:for (row ; nrows)
        const uint n = ncols * NColors;
        byte* pixline;
        pixline = &pixels[NColors * (nrows - row - 1) * ncols];

        if (!xgetn_byte_FileB (&xfb, pixline, n))
        {
            good = false;
            break;
        }

        {:for (i ; n)
            pixline[i] = (byte)
                (256 * (uint) pixline[i] / (max_color_value+1));
        }
    }

    lose_FileB (&xfb);

    if (good)
    {
        *ret_nrows = nrows;
        *ret_ncols = ncols;
    }
    else
    {
        free (pixels);
        pixels = 0;
    }

    fprintf (stderr, "Read PPM seconds:%f\n", monotime () - t0);
    return pixels;
}

