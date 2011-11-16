
#include "scene.h"

#include "order.h"

#include <assert.h>
#include <string.h>

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
midfill_between_simplices (SceneElement* dst_elems,
                           uint* nverts,
                           Point* verts,
                           uint* nvnmls,
                           Point* vnmls,
                           uint k,
                           const SceneElement* a,
                           const SceneElement* b,
                           uint a_vert_offset,
                           uint b_vert_offset,
                           uint a_vnml_offset,
                           uint b_vnml_offset);
static uint
brutefill_between_simplices (SceneElement* dst_elems,
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
    UFor( i, NDimensions )
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
copy_Scene (Scene* dst, const Scene* src)
{
    uint i;
    CopyT( Scene, dst, src, 0, 1 );
    dst->elems = DuplicaT( SceneElement, dst->elems, dst->nelems );
    dst->verts = DuplicaT( Point, dst->verts, dst->nverts );
    dst->vnmls = DuplicaT( Point, dst->vnmls, dst->nvnmls );
    dst->txpts = DuplicaT( BaryPoint, dst->txpts, dst->ntxpts );
    dst->matls = DuplicaT( Material, dst->matls, dst->nmatls );
    dst->txtrs = DuplicaT( Texture, dst->txtrs, dst->ntxtrs );

    UFor( i, dst->ntxtrs )
    {
        Texture* txtr;
        txtr = &dst->txtrs[i];
        txtr->pixels = DuplicaT( byte, txtr->pixels,
                                 3 * txtr->nrows * txtr->ncols );
    }
}

    void
output_SceneElement (FILE* out, const Scene* scene, uint ei)
{
    uint i;
    UFor( i, NDimensions )
    {
        uint vi;
        vi = scene->elems[ei].verts[i];
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
        vert_Scene (&dst->pts[i], scene, elem->verts[i]);
}

    void
setup_1elem_Scene (Scene* scene)
{
    uint i;
    init_Scene (scene);

    scene->nelems = 1;
    scene->nverts = NDimensions;
    scene->elems = AllocT( SceneElement, scene->nelems );
    scene->verts = AllocT( Point, scene->nverts );

    init_SceneElement (&scene->elems[0]);

    UFor( i, scene->nverts )
    {
        scene->elems[0].verts[i] = i;
        zero_Point (&scene->verts[i]);
        scene->verts[i].coords[i] = 1;
    }
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
                elem->verts[j] = a_vert_offset + a->verts[j];
                if (a_vnml_offset < Max_uint)
                    elem->vnmls[j] = a_vnml_offset + a->vnmls[j];
            }
            if (j >= i)
            {
                elem->verts[j+1] = b_vert_offset + b->verts[j];
                if (b_vnml_offset < Max_uint)
                    elem->vnmls[j+1] = b_vnml_offset + b->vnmls[j];
            }
        }
    }
    return k;
}

    /* This method of filling simplices of dimension /k-1/
     * only accepts triangles. 14 tetrahedra the space between,
     * utilizing 4 new vertices.
     *
     * 1 new vertex is created between the two triangles.
     * The two triangles form 2 tetrahedra using this new vertex.
     *
     * Draw 3 planes between the unconnected tetrahedra edges such that
     * we are left with 3 pyramids. For each pyramid, create a new vertex
     * on its 4-sided face, and partition out 4 tetrahedra using the new
     * vertex. For this section, we see that 3 vertices and 12 tetrahedra
     * are formed.
     */
    uint
midfill_between_simplices (SceneElement* dst_elems,
                           uint* nverts,
                           Point* verts,
                           uint* nvnmls,
                           Point* vnmls,
                           uint k,
                           const SceneElement* a,
                           const SceneElement* b,
                           uint a_vert_offset,
                           uint b_vert_offset,
                           uint a_vnml_offset,
                           uint b_vnml_offset)
{
    bool use_vnmls = true;
    uint i;
    Point* new_verts;  Point* new_vnmls;
    const uint nelems = 14;
    const uint elems[][4] =
    {
        { 0,            6, 5, 4            },
        { 0,                       7, 8, 9 },
        { 0, 1,         4, 5               },
        { 0,    2,         5, 6            },
        { 0,       3,   6,    4            },
        { 0, 1,                    8, 7    },
        { 0,    2,                    9, 8 },
        { 0,       3,              7,    9 },
        { 0, 1,         7,         4       },
        { 0,    2,         8,         5    },
        { 0,       3,         9,         6 },
        { 0, 1,            5,         8    },
        { 0,    2,            6,         9 },
        { 0,       3,   4,         7       }
    };
    uint vertidcs[10], vnmlidcs[10];

    assert (NDimensions == 4);
    assert (k == 3);
    if (NDimensions != 4 || k != 3)  return 0;

    UFor( i, 4 )  vertidcs[    i] = *nverts + i;
    UFor( i, k )  vertidcs[4+  i] = a_vert_offset + a->verts[i];
    UFor( i, k )  vertidcs[4+k+i] = b_vert_offset + b->verts[i];

    UFor( i, 4 )  vnmlidcs[    i] = *nvnmls + i;
    UFor( i, k )
    {
        vnmlidcs[4+  i] = a->vnmls[i];
        vnmlidcs[4+k+i] = b->vnmls[i];
        if (vnmlidcs[4+  i] < Max_uint)  vnmlidcs[4+  i] += a_vnml_offset;
        else  use_vnmls = false;
        if (vnmlidcs[4+k+i] < Max_uint)  vnmlidcs[4+k+i] += b_vnml_offset;
        else  use_vnmls = false;
    }

    new_verts = &verts[*nverts];
    new_vnmls = &vnmls[*nvnmls];
    *nverts += 4;
    if (use_vnmls)  *nvnmls += 4;

    UFor( i, 4 )
    {
        zero_Point (&new_verts[i]);
        zero_Point (&new_vnmls[i]);
    }

        /* k == 3 */
    UFor( i, k )
    {
        Point vertsum, vnmlsum;
        summ_Point (&vertsum,
                    &verts[a_vert_offset + a->verts[i]],
                    &verts[b_vert_offset + b->verts[i]]);
        if (use_vnmls)
            summ_Point(&vnmlsum,
                       &vnmls[a_vnml_offset + a->vnmls[i]],
                       &vnmls[b_vnml_offset + b->vnmls[i]]);
        else
            zero_Point (&vnmlsum);

        summ_Point (&new_verts[0], &new_verts[0], &vertsum);
        summ_Point (&new_vnmls[0], &new_vnmls[0], &vnmlsum);

        summ_Point (&new_verts[i+1], &new_verts[i+1], &vertsum);
        summ_Point (&new_vnmls[i+1], &new_vnmls[i+1], &vnmlsum);
        if (i == 0)
        {
            summ_Point (&new_verts[k], &new_verts[k], &vertsum);
            summ_Point (&new_vnmls[k], &new_vnmls[k], &vnmlsum);
        }
        else
        {
            summ_Point (&new_verts[i], &new_verts[i], &vertsum);
            summ_Point (&new_vnmls[i], &new_vnmls[i], &vnmlsum);
        }
    }

    scale_Point (&new_verts[0], &new_verts[0], .5 / k);
    UFor( i, k )
        scale_Point (&new_verts[i+1], &new_verts[i+1], .25);
    if (use_vnmls)
        UFor( i, 4 )
            normalize_Point (&new_vnmls[i], &new_vnmls[i]);
    
    UFor( i, nelems )
    {
        uint j;
        SceneElement* elem;
        elem = &dst_elems[i];
        init_SceneElement (elem);
        elem->material = a->material;
        UFor( j, k+1 )
        {
            elem->verts[j] = vertidcs[elems[i][j]];
            if (use_vnmls)
                elem->vnmls[j] = vnmlidcs[elems[i][j]];
            else
                elem->vnmls[j] = Max_uint;
        }
    }

    return nelems;
}

    uint
brutefill_between_simplices (SceneElement* dst_elems,
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

    UFor( i, k )  vertidcs[0+i] = a_vert_offset + a->verts[i];
    UFor( i, k )  vertidcs[k+i] = b_vert_offset + b->verts[i];
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

            elem->verts[j] = v;
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
    uint vcount = 0;  /* Vertex count.*/
    uint vnmlcount = 0;  /* Vertex normal count.*/

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
                assert (scenes[0].elems[ei].verts[pi] ==
                        scenes[i+1].elems[ei].verts[pi]);

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

    vcount = nscenes * nverts;
    dst->nverts = vcount + dst->nelems;
    dst->verts = AllocT( Point, dst->nverts );

    vnmlcount = nscenes * nvnmls;
    dst->nvnmls = vnmlcount + dst->nelems;
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
            uint x;
            SceneElement* dst_elems;
            dst_elems = &dst->elems[ecount];
            if (false)
                x = brutefill_between_simplices (dst_elems,
                                                 k,
                                                 &a->elems[ei], &b->elems[ei],
                                                 a_vert_offset, b_vert_offset,
                                                 a_vnml_offset, b_vnml_offset,
                                                 dst->verts);
            else if (true)
                    /* This method looks best.*/
                x = midfill_between_simplices (dst_elems,
                                               &vcount, dst->verts,
                                               &vnmlcount, dst->vnmls,
                                               k,
                                               &a->elems[ei], &b->elems[ei],
                                               a_vert_offset, b_vert_offset,
                                               a_vnml_offset, b_vnml_offset);
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
    dst->nverts = vcount;
    ResizeT( Point, dst->verts, dst->nverts );
    dst->nvnmls = vnmlcount;
    ResizeT( Point, dst->vnmls, dst->nvnmls );
}

    void
interpolate1_Scene (Scene* dst, real alpha,
                    const Scene* scene_a, const Scene* scene_b)
{
    uint i;
    assert (alpha >= 0);
    assert (alpha <= 1);

    copy_Scene (dst, scene_a);

    UFor( i, dst->nverts )
        Op_Point_21010( &dst->verts[i]
                        ,+, (1-alpha)*, &dst->verts[i]
                        ,   alpha*, &scene_b->verts[i] );
    UFor( i, dst->nvnmls )
        Op_Point_21010( &dst->vnmls[i]
                        ,+, (1-alpha)*, &dst->vnmls[i]
                        ,   alpha*, &scene_b->vnmls[i] );
    UFor( i, dst->ntxpts )
        Op_21010( real, NDimensions-1, dst->txpts[i].coords
                  ,+, (1-alpha)*, dst->txpts[i].coords
                  ,   alpha*, scene_b->txpts[i].coords );
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
        normalize_Point (p, &tmp);
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

    assert (minimal_unique (nmembs, indices));

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

    assert (minimal_unique (nmembs, indices));

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

    assert (minimal_unique (q, jumps));

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

    assert (minimal_unique (n, indices));
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
    if (scene->nelems > max_n)  max_n = scene->nelems;

    jumps  = AllocT( uint, max_n );
    indices = AllocT( uint, max_n );
    coords  = AllocT( real, max_n );

    if (scene->nverts > 0)
    {
        uint prev_nverts;
        prev_nverts = scene->nverts;
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
                e = &scene->elems[i].verts[dim];
                assert (*e < prev_nverts);
                *e = jumps[*e];
            }
        }
    }

    if (scene->nvnmls > 0)
    {
        uint prev_nvnmls;
        prev_nvnmls = scene->nvnmls;
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
                assert (*e < prev_nvnmls);
                *e = jumps[*e];
            }
        }
    }

    if (false && scene->nelems > 0)
    {
        Point* tmp_pts;
        tmp_pts = AllocT( Point, scene->nelems );
        UFor( i, scene->nelems )
        {
            uint dim;
            UFor( dim, NDimensions )
                tmp_pts[i].coords[dim] = scene->elems[i].verts[dim];
        }
        printf ("Before nelems:%u\n", scene->nelems);
        scene->nelems = condense_Points (scene->nelems, tmp_pts,
                                         jumps, indices, coords);
        printf ("After nelems:%u\n", scene->nelems);
        UFor( i, scene->nelems )
        {
            uint dim;
            const SceneElement* src;
            SceneElement* dst;

            src = &scene->elems[jumps[i]];
            dst = &scene->elems[i];

            UFor( dim, NDimensions )
                dst->verts[dim] = src->verts[dim];
        }
        ResizeT( SceneElement, scene->elems, scene->nelems );
        free (tmp_pts);
    }

    if (max_n > 0)
    {
        free (jumps);
        free (indices);
        free (coords);
    }
}

