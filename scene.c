
#include "scene.h"

static uint
fill_between_simplices (SceneElement* elems,
                        uint k,
                        const SceneElement* a,
                        const SceneElement* b,
                        uint a_vert_offset,
                        uint b_vert_offset);
static uint
full_fill_between_simplices (SceneElement* dst_elems,
                             uint k,
                             const SceneElement* a,
                             const SceneElement* b,
                             uint a_vert_offset,
                             uint b_vert_offset,
                             uint a_vnml_offset,
                             uint b_vnml_offset,
                             const Point* verts);

void init_Scene (Scene* scene)
{
    scene->nelems = 0;
    scene->nverts = 0;
    scene->nvnmls = 0;
    scene->ntxpts = 0;
    scene->nmatls = 0;
    scene->ntxtrs = 0;
}

void init_SceneElement (SceneElement* elem)
{
    uint i;
    elem->material = Max_uint;
    UFor( i, NDimensions-1 )
    {
        elem->vnmls[i] = Max_uint;
        elem->txpts[i] = Max_uint;
    }
}

void copy_SceneElement (SceneElement* dst, const SceneElement* src)
{
    memcpy (dst, src, sizeof (SceneElement));
}

void cleanup_Scene (Scene* scene)
{
    if (scene->nelems > 0)  free (scene->elems);
    if (scene->nverts > 0)  free (scene->verts);
    if (scene->nvnmls > 0)  free (scene->vnmls);
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



    uint
simplex_fill_count (uint k)
{
    /*
     * Binomial coefficient
     *  /   2*k   \               (2*k)!             (2*k) * (2*k-1)
     *  |         |  =  -------------------------  = --------------- = ...
     *  \ 2*(k-1) /     (2*k-2*(k-1))! * (2*k-1)!       2! * 1
     */
    return k * (2 * k -1);
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
    uint
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
    return k;
}


    uint
full_fill_between_simplices (SceneElement* dst_elems,
                             uint k,
                             const SceneElement* a,
                             const SceneElement* b,
                             uint a_vert_offset,
                             uint b_vert_offset,
                             uint a_vnml_offset,
                             uint b_vnml_offset,
                             const Point* verts)
{
    uint ecount = 0;
    uint i;
    const uint nelems = 15;
    uint elems[][4] =
    {
        { 0, 1, 2,  3       },
        { 0, 1, 2,     4    },
        { 0, 1, 2,        5 },
        { 0, 1,     3, 4    },
        { 0, 1,     3,    5 },
        { 0, 1,        4, 5 },
        { 0,    2,  3, 4    },
        { 0,    2,  3,    5 },
        { 0,    2,     4, 5 },
        { 0,        3, 4, 5 },
        {    1, 2,  3, 4    },
        {    1, 2,  3,    5 },
        {    1, 2,     4, 5 },
        {    1,     3, 4, 5 },
        {       2,  3, 4, 5 }
    };
    assert (NDimensions == 4);
    assert (k == 3);
    if (NDimensions != 4 || k != 3)  return 0;

    ecount = 0;
    UFor( i, nelems )
    {
        uint j;
        Simplex simplex;
        SceneElement* elem;

        elem = &dst_elems[ecount];
        init_SceneElement (elem);

        UFor( j, NDimensions )
        {
            uint vi, v, vn;
            vi = elems[i][j];
            if (vi < 3)
            {
                v = a_vert_offset + a->pts[vi];
                vn = a->vnmls[vi];
                if (vn < Max_uint)  vn += a_vnml_offset;
            }
            else
            {
                v = b_vert_offset + b->pts[vi-3];
                vn = b->vnmls[vi-3];
                if (vn < Max_uint)  vn += b_vnml_offset;
            }

            elem->pts[j] = v;
            elem->vnmls[j] = vn;
            copy_Point (&simplex.pts[j], &verts[v]);
        }

        if (!degenerate_Simplex (&simplex))
        {
            ecount += 1;
            elem->material = a->material;
        }
#if 0
        else
        {
            FILE* out = stderr;
            fputs ("Skipping: ", out);
            output_Simplex (out, &simplex);
            fputc ('\n', out);
        }
#endif
    }

    return ecount;
}


    /* /k/ is the dimension, see fill_between_simplices() for more info.*/
    void
interpolate_Scene (Scene* dst, uint k, uint nscenes, const Scene* scenes)
{
    uint i;
    uint nelems, nverts, nvnmls, nmatls;
    uint ecount = 0;  /* Element count.*/

    assert (nscenes > 0);
    nelems = scenes[0].nelems;
    nverts = scenes[0].nverts;
    nvnmls = scenes[0].nvnmls;
    nmatls = scenes[0].nmatls;

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
            /* Constant vertex normal counts.*/
        assert (scenes[i+1].nvnmls == nvnmls);
            /* Constant material counts (we hope they are the same).*/
        assert (scenes[i+1].nmatls == nmatls);

        UFor( ei, nelems )
        {
                /* All elements' vertex numbers are identical.*/
            UFor( pi, k )
                assert (scenes[0].elems[ei].pts[pi] ==
                        scenes[i+1].elems[ei].pts[pi]);

                /* Don't feel like interpolating materials.*/
            assert (scenes[0].elems[ei].material ==
                    scenes[i+1].elems[ei].material);
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
            /* No textures allowed.*/
            /* assert (scenes[i].ntxpts == 0); */
        assert (scenes[i].ntxtrs == 0);
    }

    init_Scene (dst);
    dst->nelems = (nscenes-1) * nelems * simplex_fill_count (k);
    dst->elems = AllocT( SceneElement, dst->nelems );
    dst->nverts = nscenes * nverts;
    dst->verts = AllocT( Point, dst->nverts );
    dst->nvnmls = nscenes * nvnmls;
    dst->vnmls = AllocT( Point, dst->nvnmls );
    dst->nmatls = nmatls;
    dst->matls = AllocT( Material, dst->nmatls );
    
        /* Copy info.*/
    UFor( i, nscenes )
    {
        CopyT( Point, dst->verts, scenes[i].verts, i * nverts, nverts );
        CopyT( Point, dst->vnmls, scenes[i].vnmls, i * nvnmls, nvnmls );
    }
    CopyT( Material, dst->matls, scenes[0].matls, 0, nmatls );

        /* Create /k/-dimensional simplices.*/
    UFor( i, nscenes-1 )
    {
        uint ei;
        const Scene* a;
        const Scene* b;
        uint a_vert_offset, b_vert_offset;
        uint a_vnml_offset, b_vnml_offset;

        a = &scenes[i];
        b = &scenes[i+1];
        a_vert_offset = i * nverts;
        b_vert_offset = (i+1) * nverts;
        a_vnml_offset = i * nvnmls;
        b_vnml_offset = (i+1) * nvnmls;

        UFor( ei, nelems )
        {
            const bool use_full_fill = true;
            uint x;
            SceneElement* dst_elems;
            dst_elems = &dst->elems[ecount];
            if (use_full_fill)
                x = full_fill_between_simplices (dst_elems, k,
                                                 &a->elems[ei], &b->elems[ei],
                                                 a_vert_offset, b_vert_offset,
                                                 a_vnml_offset, b_vnml_offset,
                                                 dst->verts);
            else
                x = fill_between_simplices (dst_elems, k,
                                            &a->elems[ei], &b->elems[ei],
                                            a_vert_offset, b_vert_offset);
            ecount += x;
        }
    }

    dst->nelems = ecount;
    ResizeT( SceneElement, dst->elems, dst->nelems );
}

