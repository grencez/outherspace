
#include "scene.h"

static uint
fill_between_simplices (SceneElement* elems,
                        uint k,
                        const SceneElement* a,
                        const SceneElement* b,
                        uint a_vert_offset,
                        uint b_vert_offset,
                        uint a_vnml_offset,
                        uint b_vnml_offset);
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
                        uint b_vert_offset,
                        uint a_vnml_offset,
                        uint b_vnml_offset)
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
            if (j <= i)
            {
                elem->pts[j] = a_vert_offset + a->pts[j];
                elem->vnmls[j] = a_vnml_offset + a->vnmls[j];
            }
            if (j >= i)
            {
                elem->pts[j+1] = b_vert_offset + b->pts[j];
                elem->vnmls[j+1] = b_vnml_offset + b->vnmls[j];
            }
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
    const uint elems[][4] =
    {
        { 0, 1, 2,  3       },
        { 0, 1, 2,     4    },
        { 0, 1, 2,        5 },
        { 0, 1,     3, 4    },
        { 0, 1, 5,  3       },
        { 0, 1, 5,     4    },
        { 0, 4, 2,  3       },
        { 0,    2,  3,    5 },
        { 0, 4, 2,        5 },
        { 0, 4, 5,  3       },
        { 3, 1, 2,     4    },
        { 3, 1, 2,        5 },
        {    1, 2,     4, 5 },
        { 3, 1, 5,     4,   },
        { 3, 4, 2,        5 }
    };
    uint vertidcs[6], vnmlidcs[6];
    assert (NDimensions == 4);
    assert (k == 3);
    if (NDimensions != 4 || k != 3)  return 0;

    UFor( i, k )  vertidcs[0+i] = a_vert_offset + a->pts[i];
    UFor( i, k )  vertidcs[k+i] = b_vert_offset + b->pts[i];
    UFor( i, k )
    {
        vnmlidcs[0+i] = a->vnmls[i];
        if (vnmlidcs[0+i] < Max_uint)  vnmlidcs[0+i] += a_vnml_offset;
    }
    UFor( i, k )
    {
        vnmlidcs[k+i] = b->vnmls[i];
        if (vnmlidcs[k+i] < Max_uint)  vnmlidcs[k+i] += b_vnml_offset;
    }

    ecount = 0;
    UFor( i, nelems )
    {
        uint j;
        Simplex simplex;
        SceneElement* elem;

        elem = &dst_elems[ecount];
        init_SceneElement (elem);

        UFor( j, k+1 )
        {
            uint vi, v;
            vi = elems[i][j];
            v = vertidcs[vi];

            elem->pts[j] = v;
            elem->vnmls[j] = vnmlidcs[vi];

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
                x = full_fill_between_simplices (dst_elems,
                                                 k,
                                                 &a->elems[ei], &b->elems[ei],
                                                 a_vert_offset, b_vert_offset,
                                                 a_vnml_offset, b_vnml_offset,
                                                 dst->verts);
            else
                x = fill_between_simplices (dst_elems, k,
                                            &a->elems[ei], &b->elems[ei],
                                            a_vert_offset, b_vert_offset,
                                            a_vnml_offset, b_vnml_offset);
            ecount += x;
        }
    }

    dst->nelems = ecount;
    ResizeT( SceneElement, dst->elems, dst->nelems );
}

    void
xlate_Scene (Scene* scene, const Point* displacement)
{
    uint i;
    UFor( i, scene->nverts )
        summ_Point (&scene->verts[i], &scene->verts[i], displacement);
}

    void
xfrm_Scene (Scene* scene, const PointXfrm* xfrm)
{
    uint i;

    UFor( i, scene->nverts )
    {
        Point* p;
        Point tmp;
        p = &scene->verts[i];
        xfrm_Point (&tmp, xfrm, p);
        copy_Point (p, &tmp);
    }

    UFor( i, scene->nvnmls )
    {
        Point* p;
        Point tmp;
        p = &scene->vnmls[i];
        xfrm_Point (&tmp, xfrm, p);
        copy_Point (p, &tmp);
    }
}

    void
recenter_Scene (Scene* scene, const Point* new_centroid)
{
    BoundingBox box;
    Point displacement;
    init_BoundingBox (&box, scene->nverts, scene->verts);
    centroid_BoundingBox (&displacement, &box);
    if (new_centroid)
        diff_Point (&displacement, new_centroid, &displacement);
    else
        negate_Point (&displacement, &displacement);
    xlate_Scene (scene, &displacement);
}

static
    void
sort_indexed_Points (uint* jumps, uint* indices, real* coords,
                     uint nmembs, const Point* pts)
{
    uint dim;

    minimal_unique (nmembs, indices);

    jumps[0] = nmembs;

    UFor( dim, NDimensions )
    {
        uint q = 0;
        while (q < nmembs)
        {
            uint i, s;

            s = jumps[q];
            for (i = q; i < s; ++i)
                coords[indices[i]] = pts[indices[i]].coords[dim];
            sort_indexed_reals (indices, q, s, coords);

            while (q < s)
            {
                uint r;
                r = consecutive_indexed_reals (q, s, indices, coords);
                jumps[q] = r;
                q = r;
            }
            assert (q == s);
        }
    }

    minimal_unique (nmembs, indices);

    if (nmembs > 0)
    {
        uint i;
        UFor( i, nmembs-1 )
            assert (ordered_Point (&pts[indices[i]], &pts[indices[i+1]]));
    }
}

static
    uint
condense_Points (uint n, Point* pts, uint* jumps, uint* indices, real* coords)
{
    uint i, q;

    UFor( i, n )  indices[i] = i;
    sort_indexed_Points (jumps, indices, coords, n, pts);

    i = 0;
    q = 0;
    while (i < n)
    {
        uint r;
        r = jumps[i];
        assert (i < r);

        jumps[i] = jumps[q];
        jumps[q] = q;
        swap_uint (&indices[q], &indices[i]);

        ++i;
        while (i < r)
        {
            assert (equal_Point (&pts[indices[q]],
                                 &pts[indices[i]]));
            jumps[i] = q;
            ++i;
        }
        ++q;
    }

    UFor( i, n )
        assert (equal_Point (&pts[indices[i]],
                             &pts[indices[jumps[i]]]));

        /* Fixup indices so points in range don't move.*/
    UFor( i, q )
    {
        uint* e;
        e = &indices[i];
        while (*e < q && *e != i)
        {
            swap_uint (&jumps[*e], &jumps[i]);
            swap_uint (&indices[*e], e);
        }
    }

    invert_jump_table (q, jumps);

    minimal_unique (q, jumps);

    for (i = q; i < n; ++i)
        assert (equal_Point (&pts[indices[i]],
                             &pts[indices[jumps[jumps[i]]]]));

        /* Fixup duplicates' jumps to indices.*/
    for (i = q; i < n; ++i)
        jumps[i] = jumps[jumps[i]];

        /* And make the uniques' jumps consistent.*/

    UFor( i, q )
        jumps[i] = i;

    UFor( i, n )
        assert (equal_Point (&pts[indices[i]],
                             &pts[indices[jumps[i]]]));

    UFor( i, q )
    {
        uint pi;
        pi = indices[i];
        if (pi != i)
        {
            assert (q <= pi);
            copy_Point (&pts[i], &pts[pi]);
        }
    }

    minimal_unique (n, indices);
    UFor( i, n )
    {
        uint pi;
        pi = indices[i];
        indices[i] = n;  /* Never get info from this location again!*/
        while (pi != i)
        {
            assert (i < pi);
            swap_uint (&jumps[i], &jumps[pi]);
            swap_uint (&pi, &indices[pi]);
        }
    }

    return q;
}

    void
condense_Scene (Scene* scene)
{
    uint i;
    uint max_n;
    uint* jumps;
    uint* indices;
    real* coords;

    max_n = scene->nverts;
    if (scene->nvnmls > max_n)  max_n = scene->nvnmls;

    jumps  = AllocT( uint, max_n );
    indices = AllocT( uint, max_n );
    coords  = AllocT( real, max_n );

    if (scene->nverts > 0)
    {
            /* printf ("Before nverts:%u\n", scene->nverts); */
        scene->nverts = condense_Points (scene->nverts, scene->verts,
                                         jumps, indices, coords);
            /* printf ("After nverts:%u\n", scene->nverts); */
        ResizeT( Point, scene->verts, scene->nverts );
        UFor( i, scene->nelems )
        {
            uint dim;
            UFor( dim, NDimensions )
            {
                uint* e;
                e = &scene->elems[i].pts[dim];
                *e = jumps[*e];
            }
        }
    }

    if (scene->nvnmls > 0)
    {
            /* printf ("Before nvnmls:%u\n", scene->nvnmls); */
        scene->nvnmls = condense_Points (scene->nvnmls, scene->vnmls,
                                         jumps, indices, coords);
            /* printf ("After nvnmls:%u\n", scene->nvnmls); */
        ResizeT( Point, scene->vnmls, scene->nvnmls );
        UFor( i, scene->nelems )
        {
            uint dim;
            UFor( dim, NDimensions )
            {
                uint* e;
                e = &scene->elems[i].vnmls[dim];
                *e = jumps[*e];
            }
        }
    }

    if (max_n > 0)
    {
        free (jumps);
        free (indices);
        free (coords);
    }
}

