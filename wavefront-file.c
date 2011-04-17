
#include "slist.h"
#include "wavefront-file.h"

#include <assert.h>
#include <string.h>

static bool
streql (const void* a, const void* b);
static bool
strto_real_colors (real* a, const char* line);
static bool
readin_materials (SList* matlist, SList* namelist, const char* filename);

    bool
streql (const void* a, const void* b)
{
    return 0 == strcmp ((char*) a, (char*) b);
}

bool readin_wavefront (RaySpace* space, const char* filename)
{
    uint line_no = 0;
    uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    SList vertlist, elemlist, matlist, matnamelist;
    uint material = Max_uint;
    Scene* scene;

    buf[len-1] = 0;

    scene = &space->scene;

    in = fopen (filename, "rb");
    if (!in)  return false;

    init_SList (&vertlist);
    init_SList (&elemlist);
    init_SList (&matlist);
    init_SList (&matnamelist);

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        uint i;
        line_no += 1;

        i = strcspn (buf, "\r\n");
        buf[i] = '\0';

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
            SceneElement elem;
            init_SceneElement (&elem);
            elem.material = material;
            good = false;

            line = &line[1];
            if (line)  line = strto_uint (&elem.pts[0], line);
            if (line)  line = strto_uint (&elem.pts[1], line);

            if (line) while (true)
            {
                SceneElement* tri_elt;
                line = strto_uint (&elem.pts[2], line);
                if (line)  good = true;
                else       break;

                tri_elt = AllocT( SceneElement, 1 );
                copy_SceneElement (tri_elt, &elem);
                app_SList (&elemlist, tri_elt);

                elem.pts[1] = elem.pts[2];
            }
        }
        else if (0 == strncmp (line, "mtllib", 6))
        {
            line = &line[6];
            i = strspn (line, " \t");
            line = &line[i];
            good = readin_materials (&matlist, &matnamelist, line);
            if (!good)
            {
                fprintf (stderr, "Line:%u  Failed to read materials!\n",
                         line_no);
            }
        }
        else if (0 == strncmp (line, "usemtl", 6))
        {
            line = &line[6];
            i = strspn (line, " \t");
            line = &line[i];
            material = search_SList (&matnamelist, line, streql);
        }
    }
    fclose (in);

    cleanup_SList (&matnamelist);

    if (!good)
    {
        cleanup_SList (&vertlist);
        cleanup_SList (&elemlist);
        cleanup_SList (&matlist);
    }
    else
    {
        uint ei;

        scene->nverts = vertlist.nmembs;
        scene->nelems = elemlist.nmembs;
        scene->nmatls = matlist.nmembs;
        scene->verts = AllocT( Point, vertlist.nmembs );
        scene->elems = AllocT( SceneElement, elemlist.nmembs );
        scene->matls = AllocT( Material, matlist.nmembs );
        unroll_SList (scene->verts, &vertlist, sizeof (Point));
        unroll_SList (scene->elems, &elemlist, sizeof (SceneElement));
        unroll_SList (scene->matls, &matlist, sizeof (Material));

        space->nelems = scene->nelems;
        space->elems = AllocT( Triangle, space->nelems );

        UFor( ei, scene->nelems )
        {
            uint pi;
            SceneElement* read_tri;
            Triangle* tri;

            read_tri = &scene->elems[ei];
            tri = &space->elems[ei];

            UFor( pi, NTrianglePoints )
            {
                uint vert_id;
                vert_id = read_tri->pts[pi];
                if (vert_id == 0 || vert_id > scene->nverts)
                {
                    fprintf (stderr, "Bad vertex:%u\n", vert_id);
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


    bool
strto_real_colors (real* a, const char* line)
{
    uint i;
    UFor( i, NColors )
    {
        if (line)  line = strto_real (&a[i], line);
        if (!line)
        {
            if (i == 0)  return false;
            a[i] = a[i-1]; 
        }
    }
    return true;
}


    bool
readin_materials (SList* matlist, SList* namelist, const char* filename)
{
    uint line_no = 0;
    uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    Material* material;

    assert (matlist->nmembs == namelist->nmembs);
    in = fopen (filename, "rb");
    if (!in)
    {
        fprintf (stderr, "Could not open file:%s\n", filename);
        return false;
    }

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        uint i;
        line_no += 1;

        i = strcspn (buf, "\r\n");
        buf[i] = '\0';

        i = strspn (line, " \t");
        assert (i < len);
        line = &line[i];

        if (0 == strncmp (line, "newmtl", 6))
        {
            uint namelen;
            char* name;

            line = &line[6];
            i = strspn (line, " \t");
            line = &line[i];

            namelen = strlen (line);
            name = AllocT( char, namelen+1 );
            memcpy (name, line, (namelen+1) * sizeof(char));
            app_SList (namelist, name);

            material = AllocT( Material, 1 );
            init_Material (material);
            app_SList (matlist, material);

        }
        else if (0 == strncmp (line, "Ns", 2))
        {
            line = strto_real (&material->shininess, &line[2]);
            if (!line)  good = false;
        }
        else if (0 == strncmp (line, "d", 1))
        {
            line = strto_real (&material->alpha, &line[1]);
            if (!line)  good = false;
        }
        else if (0 == strncmp (line, "Tf", 2))
        {
            good = strto_real_colors (material->transmission, &line[2]);
        }
        else if (0 == strncmp (line, "Ka", 2))
        {
            good = strto_real_colors (material->ambient, &line[2]);
        }
        else if (0 == strncmp (line, "Kd", 2))
        {
            good = strto_real_colors (material->diffuse, &line[2]);
        }
        else if (0 == strncmp (line, "Ks", 2))
        {
            good = strto_real_colors (material->specular, &line[2]);
        }
    }

    if (!good)
        fprintf (stderr, "Material read falied at %s:%u\n",
                 filename, line_no);

    assert (matlist->nmembs == namelist->nmembs);
    fclose (in);
    return good;
}

