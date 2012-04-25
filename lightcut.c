
#include "lightcut.h"

#include "affine.h"
#include "bbox.h"
#include "gmrand.h"
#include "kptree.h"

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

typedef struct LightCutBuild LightCutBuild;
struct LightCutBuild
{
    LightCutNode* node;
    BBox box;
    BBox dox;  /* Bounding box of direction vectors.*/
    uint idx;
    real cos_bound;
};


    void
init_LightCutTree (LightCutTree* t)
{
    InitTable( LightCutNode, t->nodes );
    init_BSTree (&t->bst, &t->sentinel, 0);
}

    void
lose_LightCutTree (LightCutTree* t)
{
    LoseTable( LightCutNode, t->nodes );
}


    /**
     * /sdsim2/ is the proportion between spatial and directional similarity.
     **/
static
    real
cost_LightCutBuild (LightCutBuild* ret_b,
                    const LightCutBuild* c0,
                    const LightCutBuild* c1,
                    real sdsim2)
{
    real mag;
    LightCutBuild b = *c1;

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
        {
            b.cos_bound = 1;
            mag = 0;
        }
        else
        {
            real x;
            b.cos_bound = (d2 - r2 + 1) / (2 * sqrt(d2));
            x = 1 - b.cos_bound;
            mag += sdsim2 * (x*x);
        }
    }

    mag *= (maxmag_Color (&c0->node->color) +
            maxmag_Color (&b.node->color));

    *ret_b = b;
    return mag;
}

static
    real
find_light_match (LightCutBuild* close,
                  const LightCutBuild* light,
                  LightCutBuild* lights,
                  const KPTree* kptree,
                  real lmag2,
                  real sdsim2)
{
    const Point* const loc = &light->node->iatt.origin;
    uint travi = 0;
    real mag2 = lmag2;
    uint i;
    real e_min = Max_real;

    close->node = 0;
    close->idx = Max_uint;

    while (Max_uint != (i = next_KPTree (kptree, loc, &travi, &mag2)))
    {
        LightCutBuild b;
        real e;
        LightCutBuild* a = &lights[i];
        while (!a->node)
            a = &lights[a->idx];

        if (a == light)
        {
            mag2 = lmag2;
            continue;
        }
        e = cost_LightCutBuild (&b, light, a, sdsim2);
        if (e < e_min)
        {
            if (lmag2 == Max_real)  lmag2 = 4 * mag2;
            e_min = e;
            *close = b;
        }
        mag2 = lmag2;
    }
    return e_min;
}

static
    void
make_light_tree (LightCutTree* t)
{
    LightCutBuild* clusters;
    const uint nlights = t->nodes.sz;
    uint nnodes = t->nodes.sz;
    real sdsim2; /* Proportion between spatial and directional similarity.*/
    KPTreeGrid kpgrid;
    KPTree kptree;

    clusters = AllocT( LightCutBuild, nlights );
    if (!clusters)  return;

    init_KPTreeGrid (&kpgrid, nlights);
    init_BSTree (&t->bst, &t->sentinel, 0);

    SizeTable( LightCutNode, t->nodes, 2*nlights-1 );
    { BLoop( i, nlights )
        LightCutBuild* c = &clusters[i];
        c->node = &t->nodes.s[i];
        c->node->bst.split[0] = t->bst.sentinel;
        c->node->bst.split[1] = t->bst.sentinel;
        c->box.min = c->box.max = c->node->iatt.origin;
        c->dox.min = c->dox.max = c->node->iatt.direct;
        c->node->cos_bound = c->cos_bound = 1;
        c->idx = i;
        set1_KPTreeGrid (&kpgrid, i, &c->node->iatt.origin);
    } BLose()

    init_KPTree (&kptree);
    build_KPTree (&kptree, &kpgrid);

    {
        Point diag;
        measure_BBox (&diag, &kpgrid.box);
            /* scale_Point (&diag, &diag, 1.0/16); */
        sdsim2 = mag2_Point (&diag);
    }

    while (true)
    {
        LightCutBuild close;
        real min = Max_real;
        LightCutBuild* c0 = 0;
        LightCutNode* node;

        { BLoop( i, nlights )
            LightCutBuild* c1 = &clusters[i];
            real mag;
            if (!c1->node)  continue;

            mag = maxmag_Color (&c1->node->color);
            if (mag < min)
            {
                min = mag;
                c0 = c1;
            }
        } BLose()

        min = Max_real;
        close = *c0;
        close.node = 0;

#if 1
        {
            real e0;
            e0 = find_light_match (&close, c0, clusters, &kptree,
                                   Max_real, sdsim2);
#if 1
            while (close.node)
            {
                LightCutBuild close_back = close;
                real e1;
                e1 = find_light_match (&close_back, &clusters[close.idx],
                                       clusters, &kptree, Max_real, sdsim2);
                Claim( close_back.node );
                if (e0 <= e1)  break;
                e0 = e1;
                c0 = &clusters[close.idx];
                close = close_back;
            }
#endif
        }

        if (false)
#endif
        { BLoop( i, nlights )
            LightCutBuild* c1 = &clusters[i];
            LightCutBuild b = *c1;
            real mag;


            if (!c1->node)  continue;
            if (c1 == c0)  continue;

            mag = cost_LightCutBuild (&b, c0, c1, sdsim2);

            if (mag < min)
            {
                min = mag;
                close = b;
            }
        } BLose()

        if (!close.node)
        {
            Claim( c0 );
            root_for_BSTree (&t->bst, &c0->node->bst);
            break;
        }

        node = &t->nodes.s[nnodes++];
        c0->node->bst.joint = &node->bst;
        close.node->bst.joint = &node->bst;
        *node = *c0->node;
        node->bst.joint = &t->sentinel;
        node->bst.split[0] = &c0->node->bst;
        node->bst.split[1] = &close.node->bst;
        summ_Color (&node->color, &node->color, &close.node->color);
        c0->box = close.box;
        c0->dox = close.dox;
        c0->cos_bound = node->cos_bound = close.cos_bound;
        node->area += close.node->area;

        c0->node = node;
        clusters[close.idx].node = 0;
        clusters[close.idx].idx = c0->idx;
    }

    Claim2( nnodes ,==, 2*nlights-1 );

    lose_KPTreeGrid (&kpgrid);
    lose_KPTree (&kptree);
}

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
        real area;

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
        area = area_Simplex (&simplex);
        scale_Color (&color, &color, area);
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

            scale_Color (&light->color, &color, (real)elems.sz / nphotons);
            light->iatt = ray;
            light->area = area;

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
                if (maxmag_Color (&color) <= Epsilon_real)  break;
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

    { BLoop( i, space->nlights )
        space->lights[i].on = false;
    } BLose()

    tree->nodes = lights;
    make_light_tree (tree);
    LoseTable( EmisElem, elems );
}

