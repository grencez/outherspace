
#include "main.h"

#include <time.h>

void random_Triangle (Triangle* elem, const BoundingBox* box)
{
    uint pi, ci;
    UFor( pi, NTrianglePoints )
    {
#if 0
        const uint dim_cutoff = 3;
#else
        const uint dim_cutoff = NDimensions;
#endif
        UFor( ci, NDimensions )
        {
            real x, lo, hi;
            lo = box->min_corner.coords[ci];
            hi = box->max_corner.coords[ci];
            if (ci < dim_cutoff)
                x = lo + (hi - lo) * ((real) rand () / RAND_MAX);
            else
                x = 0;
                /* printf ("%f\n", x); */
            elem->pts[pi].coords[ci] = x;
        }
    }
}

Triangle* random_Triangles (uint nelems, const BoundingBox* box)
{
    uint ei;
    Triangle* elems;
    elems = AllocT( Triangle, nelems );

    UFor( ei, nelems )
        random_Triangle (&elems[ei], box);
    return elems;
}


static
    void
tri_to_BarySimplex (BarySimplex* simplex, const Triangle* tri)
{
    PointXfrm raw;
    uint pi;
    UFor( pi, NTrianglePoints )
        copy_Point (&raw.pts[pi], &tri->pts[pi]);
        
    for (pi = NTrianglePoints; pi < NDimensions; ++pi)
        zero_Point (&raw.pts[pi]);

    init_BarySimplex (simplex, &raw);
}


void random_RaySpace (RaySpace* space, uint nelems)
{
    uint i;
    BoundingBox* box;

    box = &space->scene.box;

    {
        unsigned seed = 1294785237;
            /* unsigned seed = 1294968341; */
            /* seed = time (0); */
        fprintf (stderr, "Using seed: %u\n", seed);
        srand (seed);
    }

    UFor( i, NDimensions )
    {
        box->min_corner.coords[i] = 0;
        box->max_corner.coords[i] = 100;
    }

    space->nelems = nelems;

    space->elems = random_Triangles (nelems, box);

    space->scene.nverts = NTrianglePoints * nelems;
    space->scene.nelems = nelems;
    space->scene.verts = AllocT( Point, space->scene.nverts );
    space->scene.elems = AllocT( SceneTriangle, nelems );
    space->nobjects = 0;

    UFor( i, nelems )
    {
        uint pi, offset;
        offset = i * NTrianglePoints;
        UFor( pi, NTrianglePoints )
        {
            copy_Point (&space->scene.verts[pi + offset],
                        &space->elems[i].pts[pi]);
            space->scene.elems[i].pts[pi] = pi + offset;
        }
    }
    space->simplices = AllocT( BarySimplex, nelems );
    UFor( i, nelems )
        tri_to_BarySimplex (&space->simplices[i], &space->elems[i]);
}

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

