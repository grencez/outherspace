
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
    uint row;
    FILE* out;

    out = fopen (filename, "wb");
    if (!out)
    {
        fprintf (stderr, "Cannot open file for writing:%s\n", filename);
        return;
    }

    fputs ("P3\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);
    fputs ("255\n", out);

    UFor( row, nrows )
    {
        uint col;
        const byte* pixline;

        pixline = &pixels[(nrows - row - 1) * 3 * ncols];
        UFor( col, ncols )
        {
            fprintf (out, " %u %u %u",
                     pixline[3*col+0], pixline[3*col+1], pixline[3*col+2]);
        }
        fputc ('\n', out);
    }
    fclose (out);
}


    byte*
readin_PPM_image (uint* ret_nrows, uint* ret_ncols,
                  const char* pathname, const char* filename)
{
    uint row, nrows = 0, ncols = 0;
    bool good = true;
    byte* pixels;
    const char* line;
    DecloStack( FileB, in );
    uint max_color_value = 255;
    uint header_stage;
    real t0;

    t0 = monotime ();

    init_FileB (in);
    in->f = fopen_path (pathname, filename, "rb");
    if (!in->f)
    {
        fprintf (stderr, "Cannot open file for reading:%s/%s\n",
                 pathname, filename);
        return 0;
    }


    header_stage = 0;
    while (good && header_stage < 3 && getline_FileB (in))
    {
        DecloStack( FileB, olay );
        olay_FileB (olay, in);
        skipds_FileB (olay, 0);
        line = olay->buf.s;

        if (line[0] == '#')
        {
                /* Do nothing!*/
        }
        else if (header_stage == 0)
        {
            header_stage += 1;
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
        lose_FileB (in);
        return 0;
    }

    pixels = AllocT( byte, nrows * ncols * NColors );
    
    UFor( row, nrows )
    {
        uint col;
        byte* pixline;
        pixline = &pixels[NColors * (nrows - row - 1) * ncols];
        UFor( col, ncols )
        {
            uint i;
            byte* pixcell;
            pixcell = &pixline[NColors * col];
            UFor( i, NColors )
            {
                uint x;
                line = nextok_FileB (in, 0, 0);
                if (line && strto_uint (&x, line))
                {
                    pixcell[i] = (byte) (256 * x / (max_color_value+1));
                }
                else
                {
                    lose_FileB (in);
                    return 0;
                }
            }
        }
    }

    lose_FileB (in);

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

