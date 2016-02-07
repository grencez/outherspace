
#include "scene.h"

#include "affine.h"
#include "bbox.h"
#include "order.h"
#include "point.h"
#include "serial.h"
#include "simplex.h"
#include "xfrm.h"

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
    scene->ndims = NDimensions;
    scene->nelems = 0;
    scene->nsurfs = 0;
    scene->nverts = 0;
    scene->nvnmls = 0;
    scene->ntxpts = 0;
    scene->nmatls = 0;
    scene->ntxtrs = 0;
    scene->vidcs = 0;
}

void init_SceneElement (SceneElement* elem)
{
    uint i;
    elem->material = UINT_MAX;
    elem->surface = UINT_MAX;
    UFor( i, NDimensions )
    {
        elem->vnmls[i] = UINT_MAX;
        elem->txpts[i] = UINT_MAX;
    }
}

void copy_SceneElement (SceneElement* dst, const SceneElement* src)
{
    *dst = *src;
}

    void
init_GeomSurf (GeomSurf* surf)
{
    surf->nelems = 0;
    surf->vidcs_offset = UINT_MAX;
    surf->verts_offset = UINT_MAX;
    surf->vnmls_offset = UINT_MAX;
    surf->txpts_offset = UINT_MAX;
    surf->material = UINT_MAX;
}


void cleanup_Scene (Scene* scene)
{
    if (scene->nelems > 0)  free (scene->elems);
    if (scene->nsurfs > 0)  free (scene->surfs);
    if (scene->vidcs)       free (scene->vidcs);
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
  *dst = *src;
  Duplic( dst->elems, dst->elems, dst->nelems );
  Duplic( dst->surfs, dst->surfs, dst->nsurfs );
  if (dst->vidcs)
    Duplic( dst->vidcs, dst->vidcs, dst->ndims * dst->nelems );
  Duplic( dst->verts, dst->verts, dst->nverts );
  Duplic( dst->vnmls, dst->vnmls, dst->nvnmls );
  Duplic( dst->txpts, dst->txpts, dst->ntxpts );
  Duplic( dst->matls, dst->matls, dst->nmatls );
  Duplic( dst->txtrs, dst->txtrs, dst->ntxtrs );

  UFor( i, dst->ntxtrs )
    copy_Texture (&dst->txtrs[i], &dst->txtrs[i]);
}

    void
concat0_Scene (Scene* scene, Scene* src)
{
    uint i;
    Scene orig;
    orig = *scene;

    assert (scene->ndims == src->ndims);

    ConcaT( SceneElement, scene->elems, src->elems,
            scene->nelems, src->nelems );
    ConcaT( GeomSurf, scene->surfs, src->surfs,
            scene->nsurfs, src->nsurfs );
    ConcaT( Point, scene->verts, src->verts, scene->nverts, src->nverts );
    ConcaT( Point, scene->vnmls, src->vnmls, scene->nvnmls, src->nvnmls );
    ConcaT( BaryPoint, scene->txpts, src->txpts, scene->ntxpts, src->ntxpts );
    ConcaT( Material, scene->matls, src->matls, scene->nmatls, src->nmatls );
    ConcaT( Texture, scene->txtrs, src->txtrs, scene->ntxtrs, src->ntxtrs );

    UFor( i, src->nelems )
    {
        uint dim;
        SceneElement* elem;
        elem = &scene->elems[i + orig.nelems];
        UFor( dim, NDimensions )
        {
            if (elem->verts[dim] < UINT_MAX)
                elem->verts[dim] += orig.nverts;
            if (elem->vnmls[dim] < UINT_MAX)
                elem->vnmls[dim] += orig.nvnmls;
            if (elem->txpts[dim] < UINT_MAX)
                elem->txpts[dim] += orig.ntxpts;
        }
        if (elem->material < UINT_MAX)  elem->material += orig.nmatls;
        if (elem->surface < UINT_MAX)  elem->surface += orig.nsurfs;
    }

    UFor( i, src->nsurfs )
    {
        GeomSurf* surf;
        surf = &scene->surfs[i + orig.nsurfs];
        if (surf->vidcs_offset < UINT_MAX)
            surf->vidcs_offset += orig.ndims * orig.nelems;
        if (surf->verts_offset < UINT_MAX)  surf->verts_offset += orig.nverts;
        if (surf->vnmls_offset < UINT_MAX)  surf->vnmls_offset += orig.nvnmls;
        if (surf->txpts_offset < UINT_MAX)  surf->txpts_offset += orig.ntxpts;
        if (surf->material < UINT_MAX)  surf->material += orig.nmatls;
    }

    UFor( i, src->nmatls )
    {
        Material* matl;
        matl = &scene->matls[i + orig.nmatls];
        if (matl->ambient_texture < UINT_MAX)
            matl->ambient_texture += orig.ntxtrs;
        if (matl->diffuse_texture < UINT_MAX)
            matl->diffuse_texture += orig.ntxtrs;
    }

    src->ntxtrs = 0;
    cleanup_Scene (src);
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
    scene->nsurfs = 1;
    scene->nverts = NDimensions;
    AllocTo( scene->elems, scene->nelems );
    AllocTo( scene->surfs, scene->nsurfs );
    AllocTo( scene->verts, scene->nverts );

    init_GeomSurf (&scene->surfs[0]);
    scene->surfs[0].nelems = 1;
    scene->surfs[0].verts_offset = 0;

    init_SceneElement (&scene->elems[0]);
    scene->elems[0].surface = 0;

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
                if (a_vnml_offset < UINT_MAX)
                    elem->vnmls[j] = a_vnml_offset + a->vnmls[j];
            }
            if (j >= i)
            {
                elem->verts[j+1] = b_vert_offset + b->verts[j];
                if (b_vnml_offset < UINT_MAX)
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
        if (vnmlidcs[4+  i] < UINT_MAX)  vnmlidcs[4+  i] += a_vnml_offset;
        else  use_vnmls = false;
        if (vnmlidcs[4+k+i] < UINT_MAX)  vnmlidcs[4+k+i] += b_vnml_offset;
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
        UFor( j, k+1 )
        {
            elem->verts[j] = vertidcs[elems[i][j]];
            if (use_vnmls)
                elem->vnmls[j] = vnmlidcs[elems[i][j]];
            else
                elem->vnmls[j] = UINT_MAX;
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
        if (vnmlidcs[0+i] < UINT_MAX)  vnmlidcs[0+i] += a_vnml_offset;
    }
    UFor( i, k )
    {
        vnmlidcs[k+i] = b->vnmls[i];
        if (vnmlidcs[k+i] < UINT_MAX)  vnmlidcs[k+i] += b_vnml_offset;
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
    uint nelems, nsurfs, nverts, nvnmls, nmatls;
    uint ecount = 0;  /* Element count.*/
    uint vcount = 0;  /* Vertex count.*/
    uint vnmlcount = 0;  /* Vertex normal count.*/

    assert (nscenes > 0);
    nelems = scenes[0].nelems;
    nsurfs = scenes[0].nsurfs;
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
        assert (scenes[i+1].nsurfs == nsurfs);
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

                /* Don't feel like chopping up surfaces.*/
            assert (scenes[0].elems[ei].surface ==
                    scenes[i+1].elems[ei].surface);
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
    dst->ndims = k+1;
    dst->nelems = (nscenes-1) * nelems * simplex_fill_count (k);
    AllocTo( dst->elems, dst->nelems );

    dst->nsurfs = nsurfs;
    Duplic( dst->surfs, scenes[0].surfs, dst->nsurfs );

    vcount = nscenes * nverts;
    dst->nverts = vcount + dst->nelems;
    AllocTo( dst->verts, dst->nverts );

    vnmlcount = nscenes * nvnmls;
    dst->nvnmls = vnmlcount + dst->nelems;
    AllocTo( dst->vnmls, dst->nvnmls );
    dst->nmatls = nmatls;
    Duplic( dst->matls, scenes[0].matls, dst->nmatls );

        /* Copy info.*/
    UFor( i, nscenes )
    {
        Replac( &dst->verts[i * nverts], scenes[i].verts, nverts );
        Replac( &dst->vnmls[i * nvnmls], scenes[i].vnmls, nvnmls );
    }
    UFor( i, nsurfs )
        dst->surfs[i].nelems = 0;

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
            uint j;
            const SceneElement* a_elem;  const SceneElement* b_elem;
            SceneElement* dst_elems;

            a_elem = &a->elems[ei];
            b_elem = &b->elems[ei];
            dst_elems = &dst->elems[ecount];

            if (false)
                x = brutefill_between_simplices (dst_elems,
                                                 k,
                                                 a_elem, b_elem,
                                                 a_vert_offset, b_vert_offset,
                                                 a_vnml_offset, b_vnml_offset,
                                                 dst->verts);
            else if (true)
                    /* This method looks best.*/
                x = midfill_between_simplices (dst_elems,
                                               &vcount, dst->verts,
                                               &vnmlcount, dst->vnmls,
                                               k,
                                               a_elem, b_elem,
                                               a_vert_offset, b_vert_offset,
                                               a_vnml_offset, b_vnml_offset);
            else
                x = fill_between_simplices (dst_elems, k,
                                            a_elem, b_elem,
                                            a_vert_offset, b_vert_offset,
                                            a_vnml_offset, b_vnml_offset);

            ecount += x;

            UFor( j, x )
            {
                dst_elems[j].material = a_elem->material;
                dst_elems[j].surface = a_elem->surface;
            }
        }
    }

    dst->nelems = ecount;
    ResizeT( SceneElement, dst->elems, dst->nelems );
    dst->nverts = vcount;
    ResizeT( Point, dst->verts, dst->nverts );
    dst->nvnmls = vnmlcount;
    ResizeT( Point, dst->vnmls, dst->nvnmls );

    setup_surfaces_Scene (dst);
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
        mix_Point (&dst->verts[i], &dst->verts[i], &scene_b->verts[i], alpha);

    UFor( i, dst->nvnmls )
        mix_Point (&dst->vnmls[i], &dst->vnmls[i], &scene_b->vnmls[i], alpha);

    UFor( i, dst->ntxpts )
        Op_21010( real, NDimensions-1, dst->txpts[i].coords
                  ,+, (1-alpha)*, dst->txpts[i].coords
                  ,   alpha*, scene_b->txpts[i].coords );
}

    void
map_Scene (Scene* scene, const IAMap* map)
{
    uint i;
    uint prevtex = UINT_MAX;

    UFor( i, scene->nverts )
        map_Point (&scene->verts[i], map, &scene->verts[i]);

    UFor( i, scene->nvnmls )
        xfrm_Point (&scene->vnmls[i], &map->xfrm, &scene->vnmls[i]);

    UFor( i, scene->nmatls )
    {
        uint texidx;
        texidx = scene->matls[i].bump_texture;
        if (texidx < UINT_MAX && texidx != prevtex)
        {
            prevtex = texidx;
            remap_bumps_Texture (&scene->txtrs[texidx], map);
        }
    }
}

    void
recenter_Scene (IAMap* map, const Scene* scene,
                const Point* new_centroid)
{
    BBox box;
    Point centroid, displacement;

    init_BBox (&box, scene->nverts, scene->verts);
    centroid_BBox (&centroid, &box);
    map_Point (&centroid, map, &centroid);

    if (new_centroid)
        diff_Point (&displacement, new_centroid, &centroid);
    else
        negate_Point (&displacement, &centroid);
    xlat0_IAMap (map, &displacement);
}

    void
reshuffle_for_surfaces_Scene (Scene* scene)
{
    Point* verts;  Point* vnmls;  BaryPoint* txpts;
    uint surfi;
    uint elems_offset = 0;
    const uint ndims = scene->ndims;
    GeomSurf pos;

    pos.verts_offset = 0;
    pos.vnmls_offset = 0;
    pos.txpts_offset = 0;

    verts = scene->verts;
    vnmls = scene->vnmls;
    txpts = scene->txpts;
    if (scene->nelems > 0)
        AllocTo( scene->verts, ndims * scene->nelems );
    if (scene->nvnmls > 0)
        AllocTo( scene->vnmls, ndims * scene->nelems );
    if (scene->ntxpts > 0)
        AllocTo( scene->txpts, ndims * scene->nelems );

    UFor( surfi, scene->nsurfs )
    {
        GeomSurf* surf;
        SceneElement* elem;
        uint ei;

        surf = &scene->surfs[surfi];
        assert (surf->nelems > 0);
        elem = &scene->elems[elems_offset];

            /* Set the offsets, they should come in as UINT_MAX.*/
        surf->verts_offset = pos.verts_offset;
        pos.verts_offset += ndims * surf->nelems;
        if (elem->vnmls[0] < UINT_MAX)
        {
            surf->vnmls_offset = pos.vnmls_offset;
            pos.vnmls_offset += ndims * surf->nelems;
        }
        if (elem->txpts[0] < UINT_MAX)
        {
            surf->txpts_offset = pos.txpts_offset;
            pos.txpts_offset += ndims * surf->nelems;
        }

        UFor( ei, surf->nelems )
        {
            uint i;
            elem = &scene->elems[ei + elems_offset];
            elem->surface = surfi;
            elem->material = surf->material;
            UFor( i, ndims )
            {
                uint idx;
                idx = i + ei * ndims + surf->verts_offset;
                scene->verts[idx] = verts[elem->verts[i]];
                elem->verts[i] = idx;

                Claim2( (elem->vnmls[i] < UINT_MAX) ,==, (surf->vnmls_offset < UINT_MAX) );
                if (surf->vnmls_offset < UINT_MAX)
                {
                    idx = i + ei * ndims + surf->vnmls_offset;
                    scene->vnmls[idx] = vnmls[elem->vnmls[i]];
                    elem->vnmls[i] = idx;
                }

                Claim2( (elem->txpts[i] < UINT_MAX) ,==, (surf->txpts_offset < UINT_MAX) );
                if (surf->txpts_offset < UINT_MAX)
                {
                    idx = i + ei * ndims + surf->txpts_offset;
                    scene->txpts[idx] = txpts[elem->txpts[i]];
                    elem->txpts[i] = idx;
                }
            }
        }
        elems_offset += surf->nelems;
    }

    if (scene->nverts > 0)
    {
        free (verts);
        scene->nverts = pos.verts_offset;
        ResizeT( Point, scene->verts, scene->nverts );
    }
    if (scene->nvnmls > 0)
    {
        free (vnmls);
        scene->nvnmls = pos.vnmls_offset;
        ResizeT( Point, scene->vnmls, scene->nvnmls );
    }
    if (scene->ntxpts > 0)
    {
        free (txpts);
        scene->ntxpts = pos.txpts_offset;
        ResizeT( BaryPoint, scene->txpts, scene->ntxpts );
    }
}

    void
setup_surfaces_Scene (Scene* scene)
{
    uint i;
    uint nsurfs = 0;
    SceneElement* elems;
    uint* elems_offsets;

    if (scene->nelems == 0)  return;

    UFor( i, scene->nelems )
        nsurfs = max_uint (scene->elems[i].surface, nsurfs);
    ++ nsurfs;

#if 0
    scene->nsurfs = nsurfs;
    ResizeT( GeomSurf, scene->surfs, scene->nsurfs );
#else
    assert (scene->nsurfs == nsurfs);
#endif
    UFor( i, scene->nsurfs )
    {
        GeomSurf* surf;
        uint material;
        surf = &scene->surfs[i];
        material = surf->material;
        init_GeomSurf (surf);
        surf->material = material;
    }

    UFor( i, scene->nelems )
        ++ scene->surfs[scene->elems[i].surface].nelems;

    AllocTo( elems_offsets, scene->nsurfs );
    elems_offsets[0] = 0;
    UFor( i, scene->nsurfs-1 )
        elems_offsets[i+1] = elems_offsets[i] + scene->surfs[i].nelems;

        /* TODO: Perhaps use a different reordering algo here.
         * I think condense_Points() uses one.
         * We would probably need a temporary index array
         * instead of a temporary scene element array.
         */
    Duplic( elems, scene->elems, scene->nelems );
    UFor( i, scene->nelems )
    {
        uint* elems_offset;
        elems_offset = &elems_offsets[elems[i].surface];
        scene->elems[*elems_offset] = elems[i];
        *elems_offset += 1;
    }
    free (elems);

    reshuffle_for_surfaces_Scene (scene);
}

static uint
condense_lexi_surf (real* lexis, const GeomSurf* surf,
                    const Scene* scene)
{
    const uint ndims = scene->ndims;
    uint vnml_offset = UINT_MAX;
    uint txpt_offset = UINT_MAX;
    uint i, n, stride;

    stride = ndims;  /* Vertices.*/
    if (surf->vnmls_offset < UINT_MAX)
    {
        vnml_offset = stride;
        stride += ndims;  /* Normals.*/
    }
    if (surf->txpts_offset < UINT_MAX)
    {
        txpt_offset = stride;
        stride += ndims - 1;
    }
    n = surf->nelems * ndims;

    UFor( i, n )
    {
        uint j;
        real* lexi;

        lexi = &lexis[stride * i];
        if (true)
        {
            Point p = scene->verts[i + surf->verts_offset];
            UFor( j, ndims )  lexi[j] = p.coords[j];
        }

        if (vnml_offset < UINT_MAX)
        {
            Point p = scene->vnmls[i + surf->vnmls_offset];
            UFor( j, ndims )  lexi[j + vnml_offset] = p.coords[j];
        }

        if (txpt_offset < UINT_MAX)
        {
            BaryPoint p = scene->txpts[i + surf->txpts_offset];
            UFor( j, ndims-1 )  lexi[j + txpt_offset] = p.coords[j];
        }
    }
    return stride;
}

static void
apply_jumps_surf (Scene* scene,
                  const GeomSurf* surf,
                  const GeomSurf* old_surf,
                  uint elems_offset,
                  uint n, uint* jumps, uint* indices)
{
    const uint ndims = scene->ndims;
    uint i;

    UFor( i, n )
    {
        scene->verts[i + surf->verts_offset] =
            scene->verts[indices[i] + old_surf->verts_offset];

        if (surf->vnmls_offset < UINT_MAX)
            scene->vnmls[i + surf->vnmls_offset] =
                scene->vnmls[indices[i] + old_surf->vnmls_offset];

        if (surf->txpts_offset < UINT_MAX)
            scene->txpts[i + surf->txpts_offset] =
                scene->txpts[indices[i] + old_surf->txpts_offset];
    }

    shuffle_jump_table (ndims * old_surf->nelems, jumps, indices);

    UFor( i, surf->nelems )
    {
        uint dim;
        SceneElement* elem = &scene->elems[i + elems_offset];

        UFor( dim, ndims )
        {
            uint x = jumps[elem->verts[dim] - old_surf->verts_offset];
            elem->verts[dim] = x + surf->verts_offset;
            if (elem->vnmls[dim] < UINT_MAX)
                elem->vnmls[dim] = x + surf->vnmls_offset;
            if (elem->txpts[dim] < UINT_MAX)
                elem->txpts[dim] = x + surf->txpts_offset;
        }
    }

    Replac( &scene->vidcs[surf->vidcs_offset], jumps,  ndims * surf->nelems );
}

    void
condense_Scene (Scene* scene)
{
    const uint ndims = scene->ndims;
    uint surfi;
    uint max_n = 0;
    uint* jumps;   uint* indices;
    real* coords;  real* lexis;
    GeomSurf pos;

    pos.nelems = 0;
    pos.vidcs_offset = 0;
    pos.verts_offset = 0;
    pos.vnmls_offset = 0;
    pos.txpts_offset = 0;

    assert (!scene->vidcs);
    AllocTo( scene->vidcs, ndims * scene->nelems );

    UFor( surfi, scene->nsurfs )
        max_n = max_uint (max_n, scene->surfs[surfi].nelems);

        /* Max number of vertices in a surface.*/
    max_n *= ndims;

    AllocTo( jumps, max_n );
    AllocTo( indices, max_n );
    AllocTo( coords, max_n );
    AllocTo( lexis, max_n * (3 * NDimensions - 1) );

    UFor( surfi, scene->nsurfs )
    {
        GeomSurf old_surf;
        GeomSurf* surf;
        uint stride, n;

        surf = &scene->surfs[surfi];
        old_surf = *surf;

        stride = condense_lexi_surf (lexis, surf, scene);
        n = condense_lexi_reals (jumps, indices, coords,
                                 ndims * surf->nelems, stride, lexis);

        surf->vidcs_offset = pos.vidcs_offset;
        pos.vidcs_offset += ndims * surf->nelems;
        surf->verts_offset = pos.verts_offset;
        pos.verts_offset += n;
        if (surf->vnmls_offset < UINT_MAX)
        {
            surf->vnmls_offset = pos.vnmls_offset;
            pos.vnmls_offset += n;
        }
        if (surf->txpts_offset < UINT_MAX)
        {
            surf->txpts_offset = pos.txpts_offset;
            pos.txpts_offset += n;
        }

        apply_jumps_surf (scene, surf, &old_surf,
                          pos.nelems, n, jumps, indices);

        pos.nelems += surf->nelems;
    }

    scene->nverts = pos.verts_offset;
    scene->nvnmls = pos.vnmls_offset;
    scene->ntxpts = pos.txpts_offset;

    ResizeT( Point, scene->verts, scene->nverts );
    if (scene->nvnmls > 0)
        ResizeT( Point, scene->vnmls, scene->nvnmls );
    if (scene->ntxpts > 0)
        ResizeT( BaryPoint, scene->txpts, scene->ntxpts );

    if (max_n > 0)
    {
        free (jumps);
        free (indices);
        free (coords);
        free (lexis);
    }
}

