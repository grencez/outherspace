
#include "wavefront-file.h"

#include "color.h"
#include "pnm-image.h"
#include "point.h"
#include "slist.h"

#include <assert.h>
#include <string.h>

static bool
streql (const void* a, const void* b);
static bool
strto_Color (Color* a, const char* line);
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
readin_wavefront (Scene* scene, const char* pathname, const char* filename)
{
    const uint ndims = 3;
    uint line_no = 0;
    const uint len = BUFSIZ;
    char buf[BUFSIZ];
    bool good = true;
    const char* line;
    FILE* in;
    FILE* err = stderr;
    SList elemlist, surflist,
          vertlist, vnmllist, txptlist,
          matlist, matnamelist,
          texlist, texnamelist;
    GeomSurf object_surface;
    GeomSurf* surf;

    surf = &object_surface;
    init_GeomSurf (surf);

    buf[len-1] = 0;

    in = fopen_path (pathname, filename, "rb");
    if (!in)  return false;

    init_SList (&elemlist);
    init_SList (&surflist);
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

        if (line[0] == 'v' && line[1] == 'n')
        {
            Point normal;
            line = &line[2];
            zero_Point (&normal);
            UFor( i, ndims )
                if (line)  line = strto_real (&normal.coords[i], line);

            good = !!(line);
            if (good)
            {
                normalize_Point (&normal, &normal);
                app_SList (&vnmllist, DupliT( Point, &normal, 1 ));
            }
            else
            {
                fprintf (err, "Line:%u  Not enough coordinates!\n", line_no);
            }
        }
        else if (line[0] == 'v' && line[1] == 't')
        {
            BaryPoint bpoint;
            line = &line[2];
            Op_s( real, NDimensions-1, bpoint.coords , 0 );
            UFor( i, ndims-1 )
                if (line)  line = strto_real (&bpoint.coords[i], line);

            good = !!(line);
            if (good)
                app_SList (&txptlist, DupliT( BaryPoint, &bpoint, 1 ));
            else
                fprintf (err, "Line:%u  Not enough coordinates!\n", line_no);
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
                app_SList (&vertlist, DupliT( Point, &vert, 1 ));
            }
            else
            {
                fprintf (err, "Line:%u  Not enough coordinates!\n", line_no);
            }
        }
        else if (line[0] == 'f')
        {
            SceneElement elem;
            init_SceneElement (&elem);

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
                surf->nelems += 1;

                elem.verts[1] = elem.verts[2];
                elem.txpts[1] = elem.txpts[2];
            }
            if (!good)
                fprintf (err, "Line:%u  Failed to read face!\n", line_no);
        }
        else if (AccepTok( line, "mtllib" ))
        {
            line = strskip_ws (line);
            good = readin_materials (&matlist, &matnamelist,
                                     &texlist, &texnamelist,
                                     pathname, line);
            if (!good)
            {
                fprintf (err, "Line:%u  Failed to read materials!\n", line_no);
            }
        }
        else if (AccepTok( line, "usemtl" ))
        {
            line = strskip_ws (line);
            if (surf->nelems > 0)
            {
                app_SList (&surflist, DupliT( GeomSurf, surf, 1 ));
                surf->nelems = 0;
            }

            surf->material = search_SList (&matnamelist, line, streql);
        }
    }
    fclose (in);

    cleanup_SList (&matnamelist);
    cleanup_SList (&texnamelist);

    if (!good)
    {
        cleanup_SList (&elemlist);
        cleanup_SList (&surflist);
        cleanup_SList (&vertlist);
        cleanup_SList (&vnmllist);
        cleanup_SList (&txptlist);
        cleanup_SList (&matlist);
        cleanup_SList (&texlist);
    }
    else
    {
        uint ei;

        if (surf->nelems > 0)
            app_SList (&surflist, DupliT( GeomSurf, surf, 1 ));

        init_Scene (scene);
        scene->ndims = ndims;
        scene->nelems = elemlist.nmembs;
        scene->nsurfs = surflist.nmembs;
        scene->nverts = vertlist.nmembs;
        scene->nvnmls = vnmllist.nmembs;
        scene->ntxpts = txptlist.nmembs;
        scene->nmatls = matlist.nmembs;
        scene->ntxtrs = texlist.nmembs;
        scene->elems = AllocT( SceneElement, elemlist.nmembs );
        scene->surfs = AllocT( GeomSurf, surflist.nmembs );
        scene->verts = AllocT( Point, vertlist.nmembs );
        scene->vnmls = AllocT( Point, vnmllist.nmembs );
        scene->txpts = AllocT( BaryPoint, txptlist.nmembs );
        scene->matls = AllocT( Material, matlist.nmembs );
        scene->txtrs = AllocT( Texture, texlist.nmembs );
        unroll_SList (scene->elems, &elemlist, sizeof (SceneElement));
        unroll_SList (scene->surfs, &surflist, sizeof (GeomSurf));
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
                    fprintf (err, "Bad vertex:%u\n", vi);
                    good = false;
                }
            }
        }
        reshuffle_for_surfaces_Scene (scene);
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
strto_Color (Color* a, const char* line)
{
    uint i;
    UFor( i, NColors )
    {
        if (line)  line = strto_real (&a->coords[i], line);
        if (!line)
        {
            if (i == 0)  return false;
            a->coords[i] = a->coords[i-1]; 
        }
    }
    return true;
}


static void
apply_illum_Material (Material* matl, uint illum)
{
    if (illum == 0)
        zero_Color (&matl->ambient);

    if (illum < 2)
        zero_Color (&matl->specular);

    matl->reflective = (illum >= 3);

    if (illum < 4)  matl->opacity = 1;
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
    FILE* err = stderr;
    Material scrap_material;
    Material* material;
    uint illum = 1;

    material = &scrap_material;
    init_Material (material);

    assert (matlist->nmembs == namelist->nmembs);
    in = fopen_path (pathname, filename, "rb");
    if (!in)
    {
        fprintf (err, "Could not open file:%s/%s\n", pathname, filename);
        return false;
    }

    for (line = fgets (buf, len, in);
         good && line;
         line = fgets (buf, len, in))
    {
        line_no += 1;

        strstrip_eol (buf);
        line = strskip_ws (line);

        if (AccepTok( line, "newmtl" ))
        {
            uint n = strlen (line) + 1;
            apply_illum_Material (material, illum);
            line = strskip_ws (line);

            app_SList (namelist, DupliT( char, line, n ));

            material = AllocT( Material, 1 );
            init_Material (material);
            app_SList (matlist, material);
        }
        else if (AccepTok( line, "Ns" ))
        {
            line = strto_real (&material->shininess, line);
            if (!line)  good = false;
        }
        else if (AccepTok( line, "Ni" ))
        {
            line = strto_real (&material->optical_density, line);
            if (!line)  good = false;
        }
        else if (AccepTok( line, "d" ))
        {
            real dissolve = 0;
            line = strto_real (&dissolve, line);
            if (!line)  good = false;
                /* material->opacity = 1 - dissolve; */
            material->opacity = dissolve;
        }
        else if (AccepTok( line, "Tf" ))
        {
            good = strto_Color (&material->transmission, line);
        }
        else if (AccepTok( line, "Ka" ))
        {
            good = strto_Color (&material->ambient, line);
        }
        else if (AccepTok( line, "Kd" ))
        {
            good = strto_Color (&material->diffuse, line);
        }
        else if (AccepTok( line, "Ks" ))
        {
            good = strto_Color (&material->specular, line);
        }
        else if (AccepTok( line, "Ke" ))
        {
            good = strto_Color (&material->emissive, line);
        }
        else if (AccepTok( line, "map_Ka" ))
        {
            material->ambient_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (line));
            if (material->ambient_texture == Max_uint)
                good = false;
        }
        else if (AccepTok( line, "map_Kd" ))
        {
            material->diffuse_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (line));
            if (material->diffuse_texture == Max_uint)
                good = false;
        }
        else if (AccepTok( line, "map_Ks" ))
        {
            material->specular_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (line));
            if (material->specular_texture == Max_uint)
                good = false;
        }
        else if (AccepTok( line, "map_Ke" ))
        {
            material->emissive_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (line));
            if (material->emissive_texture == Max_uint)
                good = false;
        }
        else if (AccepTok( line, "map_Bump" ))
        {
            material->bump_texture =
                parse_texture (texlist, texnamelist,
                               pathname, strskip_ws (line));
            if (material->bump_texture == Max_uint)
                good = false;
        }
        else if (AccepTok( line, "illum" ))
        {
            line = strto_uint (&illum, line);
            good = !!line;
        }
    }

    if (!good)
        fprintf (err, "Material read falied at %s:%u\n", filename, line_no);
    else
        apply_illum_Material (material, illum);

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
        Texture texture;

        good = readin_Texture (&texture, pathname, filename);

        if (good)
        {
            uint n = strlen (filename);
            i = texlist->nmembs;
            app_SList (texlist, DupliT( Texture, &texture, 1 ));
            app_SList (texnamelist, DupliT( char, filename, n+1 ));
        }
    }
    return i;
}

