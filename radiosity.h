
#ifndef Radiosity_H_
#define Radiosity_H_

#include "color.h"
#include "gmrand.h"
#include "raytrace.h"
#include "simplex.h"
#include "space.h"
#include "xfrm.h"
#include "util.h"
    /* #include "hldseq.h" */
#include <assert.h>


#if 0
    void
testfn_hldseq ()
{
    uint i;
    UFor( i, 100 )
    {
        real x = asin (sqrt (phi (2, i))) * 180 / M_PI;
        real y = 360 * phi (3, i);
        fprintf (stderr, "%f %f\n", x, y);
    }
}
#endif

typedef struct EmisElem EmisElem;
struct EmisElem
{
    Color rad;
    const Simplex* simplex;
    Point normal;
};
#ifndef Table_EmisElem
#define Table_EmisElem Table_EmisElem
DeclTableT( EmisElem, EmisElem );
#endif

static
    void
init_LightCutTree (LightCutTree* t)
{
    InitTable( LightCutNode, t->nodes );
    init_BSTree (&t->bst, &t->sentinel, 0);
}

static
    void
lose_LightCutTree (LightCutTree* t)
{
    LoseTable( LightCutNode, t->nodes );
}

static
    void
make_light_tree (LightCutTree* t)
{
    typedef struct LightCutBuild
    {
        LightCutNode* node;
        BBox box;
        BBox dox;  /* Bounding box of direction vectors.*/
    } LightCutBuild;
    LightCutBuild* clusters;
    const uint nlights = t->nodes.sz;
    uint nnodes = t->nodes.sz;
    BBox scenebox;
    real sdsim2; /* Proportion between spatial and directional similarity.*/
    clusters = AllocT( LightCutBuild, nlights );
    if (!clusters)  return;

    init0_BBox (&scenebox);

    init_BSTree (&t->bst, &t->sentinel, 0);

    SizeTable( LightCutNode, t->nodes, 2*nlights-1 );
    { BLoop( i, nlights )
        LightCutBuild* c = &clusters[i];
        c->node = &t->nodes.s[i];
        c->node->bst.split[0] = t->bst.sentinel;
        c->node->bst.split[1] = t->bst.sentinel;
        c->box.min = c->box.max = c->node->iatt.origin;
        c->dox.min = c->dox.max = c->node->iatt.direct;
        adjust_BBox (&scenebox, &c->node->iatt.origin);
    } BLose()

    {
        Point diag;
        measure_BBox (&diag, &scenebox);
        sdsim2 = mag2_Point (&diag);
    }

    while (true)
    {
        LightCutBuild* close = 0;
        real min = Max_real;
        LightCutBuild* c0 = 0;
        LightCutNode* node;

        { BLoop( i, nlights )
            LightCutBuild* c1 = &clusters[i];
            real mag;
            if (!c1->node)  continue;

            mag = taximag_Color (&c1->node->color);
            if (mag < min)
            {
                min = mag;
                c0 = c1;
            }
        } BLose()

        min = Max_real;

        { BLoop( i, nlights )
            LightCutBuild* c1 = &clusters[i];
            LightCutBuild b = *c1;
            real mag;


            if (!c1->node)  continue;
            if (c1 == c0)  continue;

                /* Squared length of cluster's bounding box diagonal
                 * gets factored into /mag/.
                 */
            {
                Point diag;
                include_BBox (&b.box, &b.box, &c0->box);
                measure_BBox (&diag, &b.box);
                mag = dot_Point (&diag, &diag);
            }

                /* Cosine of half angle of bounding cone
                 * gets factored into /mag/.
                 */
            {
                Point cent, diag;
                real d2, r2;

                centroid_BBox (&cent, &b.dox);
                d2 = mag2_Point (&cent);

                include_BBox (&b.dox, &b.dox, &c0->dox);
                measure_BBox (&diag, &b.dox);
                r2 = .25 * mag2_Point (&diag);

                if (d2 < Epsilon_real)
                    mag = 0;
                else
                    mag += sdsim2 * (1 - (d2 - r2 + 1) / (2 * sqrt(d2)));
            }

            mag *= (taximag_Color (&c0->node->color) +
                    taximag_Color (&b.node->color));

            if (mag < min)
            {
                min = mag;
                close = c1;
            }
        } BLose()

        if (!close)
        {
            Claim( c0 );
            root_for_BSTree (&t->bst, &c0->node->bst);
            break;
        }

        node = &t->nodes.s[nnodes++];
        c0->node->bst.joint = &node->bst;
        close->node->bst.joint = &node->bst;
        *node = *c0->node;
        node->bst.joint = &t->sentinel;
        node->bst.split[0] = &c0->node->bst;
        node->bst.split[1] = &close->node->bst;
        summ_Color (&node->color, &node->color, &close->node->color);
        include_BBox (&c0->box, &c0->box, &close->box);
        include_BBox (&c0->dox, &c0->dox, &close->dox);

        c0->node = node;
        close->node = 0;
    }

    Claim2( nnodes ,==, 2*nlights-1 );
}

static inline
    void
cast_lights (RaySpace* space, uint nphotons, uint nbounces)
{
    LightCutTree* tree = &space->lightcuts;
    const ObjectRaySpace* const object = &space->main;
    const Scene* const scene = &object->scene;
    GMRand gmrand;
    DeclTable( EmisElem, elems );
    DeclTable( LightCutNode, lights );

    init_GMRand (&gmrand);

    { BLoop( ei, scene->nelems )
        uint matl_idx = scene->elems[ei].material;
        const Material* matl;
        if (matl_idx == Max_uint)  continue;
        matl = &scene->matls[matl_idx];
        if (taximag_Color (&matl->emissive) > 0)
        {
            DeclGrow1Table( EmisElem, elems, elem );
            elem->rad = matl->emissive;
            elem->simplex = &object->elems[ei];
            normalize_Point (&elem->normal,
                             &object->simplices[ei].plane.normal);
        }
    } BLose()


    if (elems.sz == 0)  return;

    { BLoop( photon_idx, nphotons )
        const uint elem_idx = uint_GMRand (&gmrand, elems.sz);
        const EmisElem* const elem = &elems.s[elem_idx];
        Simplex simplex = *elem->simplex;
        Color color;
        Ray ray;
        Point c;
        real x;

        c.coords[0] = real_GMRand (&gmrand);
        c.coords[1] = real_GMRand (&gmrand);
        x = c.coords[0] + c.coords[1];
        if (x > 1)
        {
            x = 2 - x;
            c.coords[0] = 1 - c.coords[0];
            c.coords[1] = 1 - c.coords[1];
        }
        c.coords[2] = 1 - x;

        zero_Point (&ray.origin);
        { BLoop( dim, NDims )
            follow_Point (&ray.origin, &ray.origin,
                          &simplex.pts[dim],
                          c.coords[dim]);
        } BLose()

        ray.direct = elem->normal;
        color = elem->rad;
        scale_Color (&color, &color, area_Simplex (&simplex));
        quot1_Color (&color, &color, M_PI);

        { BLoop( bounce_idx, nbounces )
            DeclGrow1Table( LightCutNode, lights, light );
            PointXfrm A;
            real zenith = asin (sqrt (real_GMRand (&gmrand)));
            real azimuthcc = 2 * M_PI * real_GMRand (&gmrand);
                /* real zenith = asin (sqrt (halton (2, light_idx))); */
                /* real azimuthcc = 2 * M_PI * halton (3, light_idx); */
            uint hit = Max_uint, obj = Max_uint;
            real mag = Max_real;

                /* light->on = true; */
            scale_Color (&light->color, &color, (real)elems.sz / nphotons);
            light->iatt = ray;

            zero_Point (&c);
            c.coords[FwDim] = sin (zenith) * cos(azimuthcc); 
            c.coords[RtDim] = sin (zenith) * sin(azimuthcc);
            c.coords[UpDim] = cos (zenith);

            identity_PointXfrm (&A);
            stable_orthorotate_PointXfrm (&A, &A, &ray.direct, UpDim);
            trxfrm_Point (&ray.direct, &A, &c);

            if (bounce_idx + 1 == nbounces)  break;

            cast1_RaySpace (&hit, &mag, &obj, &ray, space, Yes);

            if (hit < Max_uint)
            {
                const ObjectRaySpace* const hitobj =
                    (obj < space->nobjects)
                    ? &space->objects[obj]
                    : &space->main;
                const uint matl_idx = hitobj->scene.elems[hit].material;
                const Material* const matl = &hitobj->scene.matls[matl_idx];
                Point normal;
                real dot;

                normal = hitobj->simplices[hit].plane.normal;
                /*
                xfrm_Point (&normal, &hitobj->orientation,
                            &hitobj->simplices[hit].plane.normal);
                            */
                normalize_Point (&normal, &normal);
                dot = dot_Point (&normal, &ray.direct);

                prod_Color (&color, &color, &matl->diffuse);
                scale_Color (&color, &color, - dot);
                    /* M_PI * match_real (1, mag * mag); */

                follow_Ray (&ray.origin, &ray, mag);
                ray.direct = normal;

                { BLoop( dim, NDims )
                    point_from_basis (&simplex.pts[dim],
                                      &object->orientation,
                                      &simplex.pts[dim],
                                      &object->centroid);
                } BLose()
            }
            else
            {
                    /* fputs ("awww...\n", stderr); */
                break;
            }
        } BLose()


    } BLose()

    PackTable( LightCutNode, lights );

#if 1
    { BLoop( i, space->nlights )
        space->lights[i].on = false;
    } BLose()
#else
    space->nlights = lights.sz;
    ResizeT( PointLightSource, space->lights, 0 );

    { BLoop( light_idx, lights.sz )
        PointLightSource* light = &space->lights[light_idx];
        init_PointLightSource (light);
        light->diffuse = true;
        light->hemisphere = true;
        light->location = lights.s[light_idx].iatt.origin;
        light->direct = lights.s[light_idx].iatt.direct;
        light->intensity = lights.s[light_idx].color;
    } BLose()
#endif
    tree->nodes = lights;
    make_light_tree (tree);
    LoseTable( EmisElem, elems );
}

static
    const LightCutNode*
next_LightCutNode (const LightCutTree* tree,
                   const LightCutNode* node,
                   real errbound,
                   const RayHit* hit)
{
    const BSTNode* const sentinel = tree->bst.sentinel;
    const BSTNode* a;

    (void) hit;

    /*
    if (!node)  return &tree->nodes.s[0];
    else if (node+1 == tree->nodes.s + (tree->nodes.sz + 1) / 2)  return 0;
    else return node + 1;
    */

    if (node)
    {
        a = &node->bst;
        while (a->joint != sentinel && a->joint->split[1] == a)
            a = a->joint;
        a = a->joint;
        if (a != sentinel)  a = a->split[1];
    }
    else
    {
        a = root_of_BSTree ((BSTree*) &tree->bst);
    }

    if (a == sentinel)  return 0;

    while (a->split[0] != sentinel)
    {
        const LightCutNode* light = CastUp( LightCutNode, bst, a );
        Point diff;
        real mag2;
        real elo;
        real ehi = errbound;

        elo = taximag_Color (&light->color);

        diff_Point (&diff, &light->iatt.origin, &hit->isect);
        mag2 = dot_Point (&diff, &diff);
#if 0
        elo *= - dot_Point (&diff, &light->iatt.direct);
        ehi *= sqrt(mag2);
            /* If negative, we won't see this light! */
        if (elo <= Epsilon_real)
            return next_LightCutNode (tree, light, errbound, hit);
#endif

        ehi *= M_PI * mag2;

        if (elo <= ehi)  return light;
        a = a->split[0];
    }

    return CastUp( LightCutNode, bst, a );
}

static
    void
cast_LightCutTree (Color* ret_color, const LightCutTree* tree,
                   const Color* diffuse,
                   const RayHit* hit,
                   const RaySpace* space)
{
        /* const uint nlights = (tree->nodes.sz + 1) / 2; */
    Color color;
    const LightCutNode* node = 0;
        /* uint ni = 0; */
    zero_Color (&color);

    while ((node = next_LightCutNode (tree, node, 0.01, hit)))
    {
        Ray tolight;
        real magtolight;
        real dot, tscale;
        diff_Point (&tolight.direct, &node->iatt.origin, &hit->isect);
            /* ni ++; */

        dot = dot_Point (&tolight.direct, &hit->normal);
        if (dot <= 0)  continue;

        magtolight = magnitude_Point (&tolight.direct);
        quot1_Point (&tolight.direct, &tolight.direct, magtolight);
        tscale = dot / magtolight;

        dot = - dot_Point (&tolight.direct, &node->iatt.direct);
        if (dot <= 0)  continue;
        tscale *= dot;

        tolight.origin = hit->isect;
        if (cast_to_light (space, &tolight, hit->front, magtolight))
        {
            real dist_factor;
            dist_factor = 1 / match_real (1e-1, magtolight * magtolight);
                /* dist_factor = 1 / (magtolight * magtolight); */
            tscale *= dist_factor;
                /* Add diffuse portion.*/
            follow_Color (&color, &color, &node->color, tscale);
        }

    }
    prod_Color (ret_color, &color, diffuse);
        /* fprintf (stderr, "%u\n", ni); */
}

#endif

