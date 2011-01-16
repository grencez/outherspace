

#include <time.h>

#include "raytrace.c"

void random_Triangle (Triangle* elem)
{
    uint pi, ci;
    UFor( pi, NTrianglePoints )
    {
        UFor( ci, NDimensions )
        {
            real x;
            x = 100 * ((real) rand () / RAND_MAX);
                /* printf ("%f\n", x); */
            elem->pts[pi].coords[ci] = x;
        }
    }
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


int main ()
{
    uint i;
    KDTree tree;
    FILE* out;
#define NELEMS 20
    uint nelems   = NELEMS;
    Triangle selems[NELEMS];
    const Triangle* elems[NELEMS];
#undef NELEMS

    out = stdout;
    {
        unsigned seed = 1294785237;
            /* unsigned seed = 1294968341; */
            /* seed = time (0); */
        fprintf (out, "Using seed: %u\n", seed);
        srand (seed);
    }

    UFor( i, nelems )
    {
        uint pi, ci;
        Triangle* elem;
        elem = &selems[i];

        UFor( pi, NTrianglePoints )
        {
            UFor( ci, NDimensions )
            {
                elem->pts[pi].coords[ci] = 0;
            }
        }

        random_Triangle (elem);

        /*
        elem->pts[0].coords[0] = 10*i;
        elem->pts[0].coords[1] = 10;
        elem->pts[0].coords[2] = 0;

        elem->pts[1].coords[0] = 10*i;
        elem->pts[1].coords[1] = 0;
        elem->pts[1].coords[2] = 0;

        elem->pts[2].coords[0] = 10*i+10;
        elem->pts[2].coords[1] = 10;
        elem->pts[2].coords[2] = 0;
        */

        elems[i] = elem;
    }

    build_KDTree (&tree, nelems, elems);
        /* output_KDTree (stdout, &tree, nelems, selems); */
    {
        uint* hits;
        const uint nrows = 2000;
        const uint ncols = 2000;
        hits = (uint*) malloc (nrows * ncols * sizeof (uint));
        rays_to_hits (hits, nrows, ncols, nelems, selems, &tree);
        output_PBM_image ("out.pbm", nrows, ncols, hits, nelems);
        output_PGM_image ("out.pgm", nrows, ncols, hits, nelems);
        free (hits);
    }
#if 0
    {
        const Triangle* elem;
        Point origin;
        Point dir;
        UFor( i, NDimensions )
        {
            dir.coords[i] = tree.box.max_corner.coords[i];
            origin.coords[i] = - dir.coords[i];
                /* origin.coords[i] = 0; */
        }
        puts ("");
        elem = cast_ray (&origin, &dir, &tree);
        if (elem)
        {
            fputs ("Found element: ", out);
            output_Triangle (out, elem);
            fputc ('\n', out);
        }
        else
        {
            fputs ("No element found.\n", out);
        }
    }
#endif

    cleanup_KDTree (&tree);

    return 0;
}

