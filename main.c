
#include "main.h"

#include <time.h>

void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    uint row;
    FILE* out;

    out = fopen (filename, "w+");
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

    out = fopen (filename, "w+");
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

    out = fopen (filename, "w+");
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

