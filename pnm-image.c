
#include "pnm-image.h"


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
            if (nelems != hitline[col])
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
    uint row;
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

    UFor( row, nrows )
    {
        uint col;
        const uint* hitline;

        hitline = &hits[(nrows - row - 1) * ncols];
        UFor( col, ncols )
        {
            fprintf (out, " %u", nelems - hitline[col]);
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
readin_PPM_image (const char* filename, uint* ret_nrows, uint* ret_ncols)
{
    uint row, nrows = 0, ncols = 0;
    bool good = true;
    byte* pixels;
    const uint capacity = BUFSIZ;
    char buf[BUFSIZ];
    uint len;
    const char* line;
    FILE* in;
    uint max_color_value = 255;
    uint header_stage;
    real t0;

    t0 = monotime ();

    in = fopen (filename, "rb");
    if (!in)
    {
        fprintf (stderr, "Cannot open file for reading:%s\n", filename);
        return 0;
    }


    header_stage = 0;
    while (good && header_stage < 3)
    {
        line = fgets (buf, capacity, in);
        strstrip_eol (buf);
        line = strskip_ws (line);

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
            line = strto_uint (&nrows, line);
            if (line)
                line = strto_uint (&ncols, line);

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
        fclose (in);
        return 0;
    }

    pixels = AllocT( byte, nrows * ncols * NColors );
    
    len = capacity - 1;
    line = buf + len;
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
                if (line == buf + len)
                {
                    len = readin_whitesep (buf, in, capacity, len);
                    line = buf;
                }

                line = strto_uint (&x, line);
                if (line)
                {
                    pixcell[i] = (byte) (256 * x / (max_color_value+1));
                }
                else
                {
                    fclose (in);
                    return 0;
                }
            }
        }
    }

    fclose (in);

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

