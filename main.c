
#include "main.h"

#include <time.h>

void random_Triangle (Triangle* elem, const BoundingBox* box)
{
    uint pi, ci;
    UFor( pi, NTrianglePoints )
    {
        UFor( ci, NDimensions )
        {
            real x, lo, hi;
            lo = box->min_corner.coords[ci];
            hi = box->max_corner.coords[ci];
            x = lo + (hi - lo) * ((real) rand () / RAND_MAX);
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

void random_RaySpace (RaySpace* space, uint nelems)
{
    uint i;
    BoundingBox box;

    {
        unsigned seed = 1294785237;
            /* unsigned seed = 1294968341; */
            /* seed = time (0); */
        fprintf (stderr, "Using seed: %u\n", seed);
        srand (seed);
    }

    UFor( i, NDimensions )
    {
        box.min_corner.coords[i] = 0;
        box.max_corner.coords[i] = 100;
    }

    space->nelems = nelems;

    space->selems = random_Triangles (nelems, &box);

    space->elems = AllocT( const Triangle*, nelems );
    space->scene.nverts = NTrianglePoints * nelems;
    space->scene.nelems = nelems;
    space->scene.verts = AllocT( Point, space->scene.nverts );
    space->scene.elems = AllocT( SceneTriangle, space->scene.nelems );

    UFor( i, nelems )
    {
        uint pi, offset;
        space->elems[i] = &space->selems[i];
        offset = i * NTrianglePoints;
        UFor( pi, NTrianglePoints )
        {
            copy_Point (&space->scene.verts[pi + offset],
                        &space->selems[i].pts[pi]);
            space->scene.elems[i].pts[pi] = pi + offset;
        }
    }

    build_KDTree (&space->tree, nelems, space->elems, space->selems);
}

void output_PBM_image (const char* filename, uint nrows, uint ncols,
                       const uint* hits, uint nelems)
{
    uint row, i;
    FILE* out;

    out = fopen (filename, "w+");
    fputs ("P1\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);

    i = 0;
    UFor( row, nrows )
    {
        uint col;
        UFor( col, ncols )
        {
            if (nelems != hits[i++])
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
    uint row, i;
    FILE* out;

    out = fopen (filename, "w+");
    fputs ("P2\n", out);
    fprintf (out, "%u %u\n", ncols, nrows);
    fprintf (out, "%u\n", nelems);

    i = 0;
    UFor( row, nrows )
    {
        uint col;
        UFor( col, ncols )
        {
            fprintf (out, " %u", nelems - hits[i++]);
        }
        fputc ('\n', out);
    }
    fclose (out);
}

