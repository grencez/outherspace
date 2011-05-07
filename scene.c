
#include "scene.h"

static void
fill_between_simplices (SceneElement* elems,
                        uint k,
                        const SceneElement* a,
                        const SceneElement* b,
                        uint a_vert_offset,
                        uint b_vert_offset);

void init_Scene (Scene* scene)
{
    scene->nelems = 0;
    scene->nverts = 0;
    scene->ntxpts = 0;
    scene->nmatls = 0;
    scene->ntxtrs = 0;
}

void init_SceneElement (SceneElement* elem)
{
    uint i;
    elem->material = Max_uint;
    UFor( i, NDimensions-1 )
        elem->txpts[i] = Max_uint;
}

void copy_SceneElement (SceneElement* dst, const SceneElement* src)
{
    memcpy (dst, src, sizeof (SceneElement));
}

void cleanup_Scene (Scene* scene)
{
    if (scene->nelems > 0)  free (scene->elems);
    if (scene->nverts > 0)  free (scene->verts);
    if (scene->ntxpts > 0)  free (scene->txpts);
    if (scene->nmatls > 0)  free (scene->matls);
    if (scene->ntxtrs > 0)
    {
        uint i;
        UFor( i, scene->ntxtrs )
            free (scene->txtrs[i].pixels);
        free (scene->txtrs);
    }
}

    void
output_SceneElement (FILE* out, const Scene* scene, uint ei)
{
    uint i;
    UFor( i, NDimensions )
    {
        uint vi;
        vi = scene->elems[ei].pts[i];
        if (i == 0)  fputc ('[', out);
        else         fputc (' ', out);
        output_Point (out, &scene->verts[vi]);
    }
    fputc (']', out);
}

void vert_Scene (Point* dst, const Scene* scene, uint idx)
{
    copy_Point (dst, &scene->verts[idx]);
}

    void
simplex_Scene (Simplex* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneElement* elem;

    elem = &scene->elems[idx];

    UFor( i, NDimensions )
        vert_Scene (&dst->pts[i], scene, elem->pts[i]);
}

    void
elem_Scene (PointXfrm* dst, const Scene* scene, uint idx)
{
    uint i;
    const SceneElement* elem;

    elem = &scene->elems[idx];

    UFor( i, NDimensions )
        vert_Scene (&dst->pts[i], scene, elem->pts[i]);
}


    uint
simplex_fill_count (uint k)
{
    return k;
}

    /* Given 2 simplices of dimension /k-1/,
     * create /k/ simplices of dimension /k/
     * which fill the gap between them.
     *
     * For example, given 2 triangles (2-dimensional simplices),
     * 3 tetrahedra (3-dimensional simplices) will be created.
     * /k/ should therefore be 3.
     *
     * Note that a /k/-dimensional simplex element has /k/+1 points!
     * Also note that a ray tracer for /k/ dimensions uses simplices of /k/-1.
     */
    void
fill_between_simplices (SceneElement* elems,
                        uint k,
                        const SceneElement* a,
                        const SceneElement* b,
                        uint a_vert_offset,
                        uint b_vert_offset)
{
    uint i;
    UFor( i, k )
    {
        uint j;
        SceneElement* elem;
        elem = &elems[i];
        init_SceneElement (elem);
        UFor( j, k )
        {
            if (j <= i)  elem->pts[j]   = a_vert_offset + a->pts[j];
            if (j >= i)  elem->pts[j+1] = b_vert_offset + b->pts[j];
        }
    }
}


    /* /k/ is the dimension, see fill_between_simplices() for more info.*/
    void
interpolate_Scene (Scene* dst, uint k, uint nscenes, const Scene* scenes)
{
    uint i;
    uint nelems, nverts;

    assert (nscenes > 0);
    nelems = scenes[0].nelems;
    nverts = scenes[0].nverts;

        /* Assert some things which aren't logically necessary,
         * but are enforced so certain setup problems can be detected.
         * In other words, these are sufficient conditions for a good setup,
         * not necessary conditions.
         * Note: The algorithm does depend on these assertions being valid,
         *       though it is not difficult to remove some of this dependence.
         */
    UFor( i, nscenes-1 )
    {
        uint ei, pi;
            /* This assertion is necessary.*/
        assert (scenes[i+1].nelems == nelems);
            /* This assertion would be difficult to work around.*/
        assert (scenes[i+1].nverts == nverts);
            /* All elements' vertex numbers are identical.*/
        UFor( ei, nelems )
        {
            UFor( pi, k )
                assert (scenes[0].elems[ei].pts[pi] ==
                        scenes[i+1].elems[ei].pts[pi]);
        }
            /* The /k/+1th coordinate of each vertex has a total order
             * from scene to scene. Stronger assertions against crossing
             * hyperplanes (lines, when /k/ is 3) could be made,
             * but that's more work.
             */
        UFor( pi, nverts )
            assert (scenes[i].verts[pi].coords[k] <
                    scenes[i+1].verts[pi].coords[k]);
    }
    UFor( i, nscenes )
    {
        uint ei;
        UFor( ei, nelems )
        {
                /* Don't feel like interpolating materials.*/
            assert (scenes[i].elems[ei].material == Max_uint);
        }
    }

    init_Scene (dst);
    dst->nelems =  (nscenes-1) * nelems * simplex_fill_count (k);
    dst->elems = AllocT( SceneElement, dst->nelems );
    dst->nverts = nscenes * nverts;
    dst->verts = AllocT( Point, dst->nverts );
    
        /* Create /k/-dimensional simplices.*/
    UFor( i, nscenes-1 )
    {
        uint ei, elem_offset;
        const Scene* a;
        const Scene* b;
        uint a_vert_offset, b_vert_offset;

        elem_offset =  i * k * nelems;
        a = &scenes[i];
        b = &scenes[i+1];
        a_vert_offset = i * nverts;
        b_vert_offset = (i+1) * nverts;

        UFor( ei, nelems )
        {
            SceneElement* dst_elems;
            dst_elems = &dst->elems[elem_offset + ei * k];
            fill_between_simplices (dst_elems, k,
                                    &a->elems[ei], &b->elems[ei],
                                    a_vert_offset, b_vert_offset);
        }
    }

        /* Copy all vertices.*/
    UFor( i, nscenes )
    {
        uint vi, vert_offset;
        vert_offset = i * nverts;
        UFor( vi, nverts )
        {
            copy_Point (&dst->verts[vert_offset + vi],
                        &scenes[i].verts[vi]);
        }
    }
}

