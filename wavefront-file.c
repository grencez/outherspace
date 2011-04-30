
#include "wavefront-file.h"

#include "pnm-image.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

static bool
streql (const void* a, const void* b);
static bool
strto_real_colors (real* a, const char* line);
static bool
readin_materials (SList* matlist, SList* namelist,
                  SList* texlist, SList* texnamelist,
                  const char* filename);

static const char*
parse_face_field (uint* v, uint* vt, uint* vn, const char* line);

static uint
parse_texture (SList* texlist, SList* texnamelist, const char* filename);

    bool
streql (const void* a, const void* b)
{
    return 0 == strcmp ((char*) a, (char*) b);
}

bool readin_wavefront (RaySpace* space, const char* filename)
{
    uint line_no = 0;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    SList elemlist, vertlist, txptlist,
          matlist, matnamelist,
          texlist, texnamelist;
    uint material = Max_uint;
    Scene* scene;

    buf[len-1] = 0;

    scene = &space->scene;

    in = fopen (filename, "rb");
    if (!in)  return false;

    init_SList (&elemlist);
    init_SList (&vertlist);
    init_SList (&txptlist);
    init_SList (&matlist);
    init_SList (&matnamelist);
    init_SList (&texlist);
    init_SList (&texnamelist);

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        uint i;
        line_no += 1;

        strstrip_eol (buf);
        line = strskip_ws (line);

        if (0 == strncmp (line, "vn", 2))
        {
        }
        else if (0 == strncmp (line, "vt", 2))
        {
            BaryPoint* point;
            point = AllocT( BaryPoint, 1 );
            app_SList (&txptlist, point);
            line = &line[2];
            UFor( i, NDimensions-1 )
            {
                line = strto_real (&point->coords[i], line);
                if (!line)
                {
                    good = false;
                    fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                             line_no);
                    break;
                }
            }
        }
        else if (line[0] == 'v')
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
            uint tmp;
            SceneElement elem;
            init_SceneElement (&elem);
            elem.material = material;
            good = false;

            line = &line[1];
            line = parse_face_field (&elem.pts[0], &elem.txpts[0], &tmp, line);
            line = parse_face_field (&elem.pts[1], &elem.txpts[1], &tmp, line);

            if (line) while (true)
            {
                SceneElement* tri_elt;
                line = parse_face_field (&elem.pts[2], &elem.txpts[2],
                                         &tmp, line);
                if (line)  good = true;
                else       break;

                tri_elt = AllocT( SceneElement, 1 );
                copy_SceneElement (tri_elt, &elem);
                app_SList (&elemlist, tri_elt);

                elem.pts[1] = elem.pts[2];
                elem.txpts[1] = elem.txpts[2];
            }
        }
        else if (0 == strncmp (line, "mtllib", 6))
        {
            line = strskip_ws (&line[6]);
            good = readin_materials (&matlist, &matnamelist,
                                     &texlist, &texnamelist, line);
            if (!good)
            {
                fprintf (stderr, "Line:%u  Failed to read materials!\n",
                         line_no);
            }
        }
        else if (0 == strncmp (line, "usemtl", 6))
        {
            line = strskip_ws (&line[6]);
            material = search_SList (&matnamelist, line, streql);
        }
    }
    fclose (in);

    cleanup_SList (&matnamelist);
    cleanup_SList (&texnamelist);

    if (!good)
    {
        cleanup_SList (&elemlist);
        cleanup_SList (&vertlist);
        cleanup_SList (&txptlist);
        cleanup_SList (&matlist);
        cleanup_SList (&texlist);
    }
    else
    {
        uint ei;

        scene->nelems = elemlist.nmembs;
        scene->nverts = vertlist.nmembs;
        scene->ntxpts = txptlist.nmembs;
        scene->nmatls = matlist.nmembs;
        scene->ntxtrs = texlist.nmembs;
        scene->elems = AllocT( SceneElement, elemlist.nmembs );
        scene->verts = AllocT( Point, vertlist.nmembs );
        scene->txpts = AllocT( BaryPoint, txptlist.nmembs );
        scene->matls = AllocT( Material, matlist.nmembs );
        scene->txtrs = AllocT( Texture, texlist.nmembs );
        unroll_SList (scene->elems, &elemlist, sizeof (SceneElement));
        unroll_SList (scene->verts, &vertlist, sizeof (Point));
        unroll_SList (scene->txpts, &txptlist, sizeof (BaryPoint));
        unroll_SList (scene->matls, &matlist, sizeof (Material));
        unroll_SList (scene->txtrs, &texlist, sizeof (Texture));

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
                if (vert_id >= scene->nverts)
                {
                    fprintf (stderr, "Bad vertex:%u\n", vert_id);
                    good = false;
                }
                else
                    copy_Point (&tri->pts[pi], &scene->verts[vert_id]);
            }
        }

        if (good)
        {
            space->simplices = AllocT( BarySimplex, space->nelems );
            UFor( ei, scene->nelems )
            {
                PointXfrm raw;
                elem_Scene (&raw, scene, ei);
                init_BarySimplex (&space->simplices[ei], &raw);
            }
            init_BoundingBox (&scene->box, scene->nverts, scene->verts);
        }
        else
        {
            free (space->elems);
        }
    }

    return good;
}


    const char*
parse_face_field (uint* v, uint* vt, uint* vn, const char* line)
{
    if (line)
    {
        line = strto_uint (v, line);
    }
    if (line)
    {
        *v -= 1;
        if (line[0] == '/')
            line = strto_uint (vt, &line[1]);
    }
    if (line)
    {
        *vt -= 1;
        if (line[0] == '/')
            line = strto_uint (vn, &line[1]);
    }
    if (line)
    {
        *vn -= 1;
    }

    return line;
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
readin_materials (SList* matlist, SList* namelist,
                  SList* texlist, SList* texnamelist,
                  const char* filename)
{
    uint line_no = 0;
    uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    Material scrap_material;
    Material* material;

    material = &scrap_material;
    init_Material (material);

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
        line_no += 1;

        strstrip_eol (buf);
        line = strskip_ws (line);

        if (0 == strncmp (line, "newmtl", 6))
        {
            uint namelen;
            char* name;

            line = strskip_ws (&line[6]);

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
        else if (0 == strncmp (line, "map_Ka", 6))
        {
            material->ambient_texture =
                parse_texture (texlist, texnamelist, strskip_ws (&line[6]));
            if (material->ambient_texture == Max_uint)
                good = false;
        }
        else if (0 == strncmp (line, "map_Kd", 6))
        {
            material->diffuse_texture =
                parse_texture (texlist, texnamelist, strskip_ws (&line[6]));
            if (material->diffuse_texture == Max_uint)
                good = false;
        }
    }

    if (!good)
        fprintf (stderr, "Material read falied at %s:%u\n",
                 filename, line_no);

    assert (matlist->nmembs == namelist->nmembs);
    fclose (in);
    return good;
}


    uint
parse_texture (SList* texlist, SList* texnamelist, const char* filename)
{
    uint i;
    i = search_SList (texnamelist, filename, streql);

    if (i == Max_uint)
    {
        Texture* texture;
        texture = AllocT( Texture, 1 );

        texture->pixels = readin_PPM_image (filename,
                                            &texture->nrows,
                                            &texture->ncols);

        if (texture->pixels)
        {
            char* s;
            i = texlist->nmembs;
            s = AllocT( char, strlen(filename) + 1 );
            strcpy (s, filename);
            app_SList (texlist, texture);
            app_SList (texnamelist, s);
        }
        else
        {
            free (texture);
        }
    }
    return i;
}

