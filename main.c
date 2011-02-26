
#include "main.h"
#include "slist.h"

#include <assert.h>
#include <string.h>
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
    space->scene.elems = AllocT( SceneTriangle, space->scene.nelems );

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

bool readin_wavefront (RaySpace* space, const char* filename)
{
    uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    SList vertlist, elemlist;
    Scene* scene;

    buf[len-1] = 0;

    scene = &space->scene;

    in = fopen (filename, "rb");
    if (!in)  return false;

    init_SList (&vertlist);
    init_SList (&elemlist);

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        uint i;
        line = strpbrk (buf, "#fv");
        if (!line)  continue;
        if (line[0] == '#')  continue;

        if (line[0] == 'v')
        {
            Point* vert;
            vert = AllocT( Point, 1 );
            app_SList (&vertlist, vert);
            line = &line[1];
            UFor( i, NDimensions )
            {
                line = strto_real (&vert->coords[i], line);
                if (!line)
                {
                    good = false;
                    break;
                }
            }
        }
        else
        {
            SceneTriangle* tri;
            assert (line[0] == 'f');
            tri = AllocT( SceneTriangle, 1 );
            app_SList (&elemlist, tri);
            line = &line[1];
            UFor( i, NTrianglePoints )
            {
                line = strto_uint (&tri->pts[i], line);
                if (!line)
                {
                    good = false;
                    break;
                }
            }
        }
    }
    fclose (in);

    if (!good)
    {
        cleanup_SList (&vertlist);
        cleanup_SList (&elemlist);
    }
    else
    {
        uint ei;

        scene->nverts = vertlist.nmembs;
        scene->nelems = elemlist.nmembs;
        scene->verts = AllocT( Point, vertlist.nmembs );
        scene->elems = AllocT( SceneTriangle, elemlist.nmembs );
        unroll_SList (scene->verts, &vertlist, sizeof (Point));
        unroll_SList (scene->elems, &elemlist, sizeof (SceneTriangle));

        space->nelems = scene->nelems;
        space->elems = AllocT( Triangle, space->nelems );

        UFor( ei, scene->nelems )
        {
            uint pi;
            SceneTriangle* read_tri;
            Triangle* tri;

            read_tri = &scene->elems[ei];
            tri = &space->elems[ei];

            UFor( pi, NTrianglePoints )
            {
                uint vert_id;
                vert_id = read_tri->pts[pi];
                if (vert_id == 0 || vert_id > scene->nelems)
                    good = false;
                else
                    copy_Point (&tri->pts[pi], &scene->verts[vert_id-1]);
            }
        }

        if (!good)
            free (space->elems);
    }

    if (good)
        init_BoundingBox (&scene->box, scene->nverts, scene->verts);

    return good;
}

