
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
                  const char* pathname, const char* filename);

static const char*
parse_face_field (uint* v, uint* vt, uint* vn, const char* line);

static uint
parse_texture (SList* texlist, SList* texnamelist,
               const char* pathname, const char* filename);

    bool
streql (const void* a, const void* b)
{
    return 0 == strcmp ((char*) a, (char*) b);
}

    bool
output_wavefront (const Scene* scene,
                  const char* pathname,
                  const char* filename)
{
    uint i;
    FILE* out;

    out = fopen_path (pathname, filename, "wb");
    if (!out)  return false;

    UFor( i, scene->nverts )
        fprintf (out, "v %f %f %f\n",
                 scene->verts[i].coords[0],
                 scene->verts[i].coords[1],
                 scene->verts[i].coords[2]);
    UFor( i, scene->ntxpts )
        fprintf (out, "vt %f %f\n",
                 scene->txpts[i].coords[0],
                 scene->txpts[i].coords[1]);
    UFor( i, scene->nvnmls )
        fprintf (out, "vn %f %f %f\n",
                 scene->vnmls[i].coords[0],
                 scene->vnmls[i].coords[1],
                 scene->vnmls[i].coords[2]);
    UFor( i, scene->nelems )
    {
        char buf[1024];
        const SceneElement* elem;
        uint j, off = 0;

        elem = &scene->elems[i];

        buf[off++] = 'f';
        buf[off++] = ' ';

        UFor( j, 3 )
        {
            uint a, b, c;
            a = elem->verts[j];
            b = elem->txpts[j];
            c = elem->vnmls[j];
            assert (a < Max_uint);
            if (b < Max_uint)
            {
                if (c < Max_uint)
                    off += sprintf (&buf[off], "%u/%u/%u", a+1, b+1, c+1);
                else
                    off += sprintf (&buf[off], "%u/%u/", a+1, b+1);
            }
            else if (c < Max_uint)
            {
                off += sprintf (&buf[off], "%u//%u", a+1, c+1);
            }
            else
            {
                off += sprintf (&buf[off], "%u", a+1);
            }
        }
        buf[off++] = '\n';
        off = fwrite (buf, sizeof(char), off, out);
        assert (off > 0);
    }
    fclose (out);
    return true;
}

    bool
readin_wavefront (Scene* scene, const AffineMap* map,
                  const char* pathname, const char* filename)
{
    const uint ndims = 3;
    uint line_no = 0;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    SList elemlist, vertlist,
          vnmllist, txptlist,
          matlist, matnamelist,
          texlist, texnamelist;
    uint material = Max_uint;

    buf[len-1] = 0;

    in = fopen_path (pathname, filename, "rb");
    if (!in)  return false;

    init_SList (&elemlist);
    init_SList (&vertlist);
    init_SList (&vnmllist);
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
            Point normal;
            line = &line[2];
            zero_Point (&normal);
            UFor( i, ndims )
                if (line)  line = strto_real (&normal.coords[i], line);

            good = !!(line);
            if (good)
            {
                mapvec_Point (&normal, map, &normal);
                normalize_Point (&normal, &normal);
                app_SList (&vnmllist, DuplicaT( Point, &normal, 1 ));
            }
            else
            {
                fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                         line_no);
            }
        }
        else if (0 == strncmp (line, "vt", 2))
        {
            BaryPoint bpoint;
            line = &line[2];
            Op_s( real, NDimensions-1, bpoint.coords , 0 );
            UFor( i, ndims-1 )
                if (line)  line = strto_real (&bpoint.coords[i], line);

            good = !!(line);
            if (good)
                app_SList (&txptlist, DuplicaT( BaryPoint, &bpoint, 1 ));
            else
                fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                             line_no);
        }
        else if (line[0] == 'v')
        {
            Point vert;
            line = &line[1];
            zero_Point (&vert);
            UFor( i, ndims )
                if (line)  line = strto_real (&vert.coords[i], line);

            good = !!(line);
            if (good)
            {
                map_Point (&vert, map, &vert);
                app_SList (&vertlist, DuplicaT( Point, &vert, 1 ));
            }
            else
            {
                fprintf (stderr, "Line:%u  Not enough coordinates!\n",
                         line_no);
            }
        }
        else if (line[0] == 'f')
        {
            SceneElement elem;
            init_SceneElement (&elem);
            elem.material = material;
            good = false;

            line = &line[1];
            line = parse_face_field (&elem.verts[0], &elem.txpts[0],
                                     &elem.vnmls[0], line);
            line = parse_face_field (&elem.verts[1], &elem.txpts[1],
                                     &elem.vnmls[1], line);

            if (line) while (true)
            {
                SceneElement* tri_elt;
                line = parse_face_field (&elem.verts[2], &elem.txpts[2],
                                         &elem.vnmls[2], line);
                if (line)  good = true;
                else       break;

                tri_elt = AllocT( SceneElement, 1 );
                copy_SceneElement (tri_elt, &elem);
                app_SList (&elemlist, tri_elt);

                elem.verts[1] = elem.verts[2];
                elem.txpts[1] = elem.txpts[2];
            }
            if (!good)
                fprintf (stderr, "Line:%u  Failed to read face!\n", line_no);
        }
        else if (0 == strncmp (line, "mtllib", 6))
        {
            line = strskip_ws (&line[6]);
            good = readin_materials (&matlist, &matnamelist,
                                     &texlist, &texnamelist,
                                     pathname, line);
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
        cleanup_SList (&vnmllist);
        cleanup_SList (&txptlist);
        cleanup_SList (&matlist);
        cleanup_SList (&texlist);
    }
    else
    {
        uint ei;

        scene->nelems = elemlist.nmembs;
        scene->nverts = vertlist.nmembs;
        scene->nvnmls = vnmllist.nmembs;
        scene->ntxpts = txptlist.nmembs;
        scene->nmatls = matlist.nmembs;
        scene->ntxtrs = texlist.nmembs;
        scene->elems = AllocT( SceneElement, elemlist.nmembs );
        scene->verts = AllocT( Point, vertlist.nmembs );
        scene->vnmls = AllocT( Point, vnmllist.nmembs );
        scene->txpts = AllocT( BaryPoint, txptlist.nmembs );
        scene->matls = AllocT( Material, matlist.nmembs );
        scene->txtrs = AllocT( Texture, texlist.nmembs );
        unroll_SList (scene->elems, &elemlist, sizeof (SceneElement));
        unroll_SList (scene->verts, &vertlist, sizeof (Point));
        unroll_SList (scene->vnmls, &vnmllist, sizeof (Point));
        unroll_SList (scene->txpts, &txptlist, sizeof (BaryPoint));
        unroll_SList (scene->matls, &matlist, sizeof (Material));
        unroll_SList (scene->txtrs, &texlist, sizeof (Texture));

        UFor( ei, scene->nelems )
        {
            const uint nverts = 3;
            uint pi;
            SceneElement* elem;
            elem = &scene->elems[ei];

            UFor( pi, nverts )
            {
                uint vi;
                vi = elem->verts[pi];
                if (vi >= scene->nverts)
                {
                    fprintf (stderr, "Bad vertex:%u\n", vi);
                    good = false;
                }
            }
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
        {
            const char* tmpline;
            tmpline = strto_uint (vt, &line[1]);
                /* Skip texture coord if need be.*/
            if (tmpline)  line = tmpline;
            else          line = &line[1];
        }
    }
    if (line)
    {
            /* Leave as Max_uint if not specified.*/
        if (*vt < Max_uint)  *vt -= 1;
        if (line[0] == '/')
            line = strto_uint (vn, &line[1]);
    }
    if (line)
    {
            /* Leave as Max_uint if not specified.*/
        if (*vn < Max_uint)  *vn -= 1;
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
                  const char* pathname, const char* filename)
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
    in = fopen_path (pathname, filename, "rb");
    if (!in)
    {
        fprintf (stderr, "Could not open file:%s/%s\n", pathname, filename);
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
        else if (0 == strncmp (line, "Ni", 2))
        {
            line = strto_real (&material->optical_density, &line[2]);
            if (!line)  good = false;
        }
        else if (0 == strncmp (line, "d", 1))
        {
            line = strto_real (&material->opacity, &line[1]);
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
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (&line[6]));
            if (material->ambient_texture == Max_uint)
                good = false;
        }
        else if (0 == strncmp (line, "map_Kd", 6))
        {
            material->diffuse_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (&line[6]));
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
parse_texture (SList* texlist, SList* texnamelist,
               const char* pathname, const char* filename)
{
    uint i;
    i = search_SList (texnamelist, filename, streql);

    if (i == Max_uint)
    {
        bool good;
        Texture* texture;
        texture = AllocT( Texture, 1 );

        good = readin_Texture (texture, pathname, filename);

        if (good)
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

