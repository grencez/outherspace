
#include "slist.h"
#include "wavefront-file.h"

#include <assert.h>
#include <string.h>

bool readin_wavefront (RaySpace* space, const char* filename)
{
    uint line_no = 0;
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
        line_no += 1;
        i = strspn (line, " \t");
        assert (i < len);
        line = &line[i];

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
                    fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                             line_no);
                    break;
                }
            }
        }
        else if (line[0] == 'f')
        {
            SceneTriangle tri;
            good = false;

            line = &line[1];
            if (line)  line = strto_uint (&tri.pts[0], line);
            if (line)  line = strto_uint (&tri.pts[1], line);

            if (line) while (true)
            {
                SceneTriangle* tri_elt;
                line = strto_uint (&tri.pts[2], line);
                if (line)  good = true;
                else       break;

                tri_elt = AllocT( SceneTriangle, 1 );
                copy_SceneTriangle (tri_elt, &tri);
                app_SList (&elemlist, tri_elt);

                tri.pts[1] = tri.pts[2];
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
                if (vert_id == 0 || vert_id > scene->nverts)
                {
                    fprintf (stderr, "Bad vertex: %u\n", vert_id);
                    good = false;
                }
                else
                    copy_Point (&tri->pts[pi], &scene->verts[vert_id-1]);
            }
            
        }

        if (good)
        {
            space->simplices = AllocT( BarySimplex, space->nelems );
            UFor( ei, scene->nelems )
                tri_to_BarySimplex (&space->simplices[ei], &space->elems[ei]);
            init_BoundingBox (&scene->box, scene->nverts, scene->verts);
        }
        else
        {
            free (space->elems);
        }
    }

    return good;
}

