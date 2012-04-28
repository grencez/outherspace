
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
    real area;
    uint nphotons;
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
    uint remidx;  /* Index into remlights table.*/
    real cos_bound;
};


    void
init_LightCutTree (LightCutTree* t)
{
    InitTable( LightCutNode, t->nodes );
    t->sentinel.nlights = 0;
    init_BSTree (&t->bst, &t->sentinel.bst, 0);
    t->area = 0;
}

    void
lose_LightCutTree (LightCutTree* t)
{
    LoseTable( LightCutNode, t->nodes );
}


    /**
     * /sdsim2/ is the proportion between spatial and directional similarity.
     * The result /ret_b/ will be a copy of /c1/ with properties which are
     * essential to computing cost combined (but not color).
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
    Color color;

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

    summ_Color (&color, &c0->node->color, &b.node->color);
    mag *= maxmag_Color (&color);

    *ret_b = b;
    return mag;
}

static
    uint
climb_LightCutBuild (LightCutBuild* a, LightCutBuild* lights)
{
    if (a->node)  return a->idx;
    return (a->idx = climb_LightCutBuild (&lights[a->idx], lights));
}

static
    real
maxmag2_LightCutBuild (const LightCutBuild* light, real e_min)
{
    Point diag;
    real min, max;
    real a2;
    real mag2;

    if (e_min == Max_real)  return Max_real;

    a2 = e_min / maxmag_Color (&light->node->color);

    measure_BBox (&diag, &light->box);
    prod_Point (&diag, &diag, &diag);
    min = max = diag.coords[NDims-1];
    mag2 = 0;
    { BLoop( dim, NDims-1 )
        real x = diag.coords[dim];
        real z = x;
        if (x < min)
        {
            z = min;
            min = x;
        }
        else if (x > max)
        {
            max = x;
        }
        mag2 += z;
    } BLose()

    mag2 = a2 - mag2;
    if (max > mag2)
        mag2 = max;

    return mag2 / 4;
}

static
    real
find_light_match (LightCutBuild* close,
                  const LightCutBuild* light,
                  LightCutBuild* lights,
                  const KPTree* kptree,
                  real e_min,
                  real sdsim2)
{
    Point loc;
    uint travi = 0;
    real lmag2 = maxmag2_LightCutBuild (light, e_min);
    real mag2 = lmag2;
    uint i;

    centroid_BBox (&loc, &light->box);

    close->node = 0;
    close->idx = Max_uint;

    while (Max_uint != (i = next_KPTree (kptree, &loc, &travi, &mag2)))
    {
        LightCutBuild b;
        real e;
        LightCutBuild* a = &lights[i];
        if (!a->node)
            a = &lights[climb_LightCutBuild (a, lights)];

        if (a == light)
        {
            mag2 = lmag2;
            continue;
        }
        e = cost_LightCutBuild (&b, light, a, sdsim2);
        if (e < e_min)
        {
            e_min = e;
            *close = b;
            lmag2 = maxmag2_LightCutBuild (light, e_min);
        }
        mag2 = lmag2;
    }
    return e_min;
}

static
    LightCutNode*
linearize_LightCutNode (LightCutNode* light,
                        LightCutTree* t,
                        LightCutNode* off)
{
    LightCutNode* sp;
    if (light == &t->sentinel)  return off;

    light->lights = off;

    sp = CastUp( LightCutNode, bst, light->bst.split[0] );
    off = linearize_LightCutNode (sp, t, off);
    light->nlights = sp->nlights;

    sp = CastUp( LightCutNode, bst, light->bst.split[1] );
    off = linearize_LightCutNode (sp, t, off);
    light->nlights += sp->nlights;

    if (light->nlights > 0)  return off;

        /* We have a single light (leaf)!*/
    light->nlights = 1;

        /* Swap positions in array.*/
    light->bst.joint->split[side_BSTNode (&light->bst)] = &off->bst;
    off->bst.joint->split[side_BSTNode (&off->bst)] = &light->bst;
    {
        LightCutNode tmp = *light;
        *light = *off;
        light = off;
        *light = tmp;
    }

    return &off[1];
}

static
    void
linearize_LightCutTree (LightCutTree* t)
{
    LightCutNode* light =
        CastUp( LightCutNode, bst, root_of_BSTree (&t->bst) );
    linearize_LightCutNode (light, t, t->nodes.s);
}

static
    void
make_light_tree (LightCutTree* t, GMRand* gmrand)
{
    LightCutBuild* clusters;
    const uint nlights = t->nodes.sz;
    uint nnodes = t->nodes.sz;
    real sdsim2; /* Proportion between spatial and directional similarity.*/
    KPTreeGrid kpgrid;
    KPTree kptree;
    DeclTable( uint, remlights );

    clusters = AllocT( LightCutBuild, nlights );
    if (!clusters)  return;

    init_KPTreeGrid (&kpgrid, nlights);
    init_BSTree (&t->bst, &t->sentinel.bst, 0);

    SizeTable( LightCutNode, t->nodes, 2*nlights-1 );
    SizeTable( uint, remlights, nlights );
    { BLoop( i, nlights )
        LightCutBuild* c = &clusters[i];
        c->node = &t->nodes.s[i];
        c->node->bst.split[0] = t->bst.sentinel;
        c->node->bst.split[1] = t->bst.sentinel;
        c->box.min = c->box.max = c->node->iatt.origin;
        c->node->bbox = c->box;
        c->dox.min = c->dox.max = c->node->iatt.direct;
        c->node->cos_bound = c->cos_bound = 1;
        c->node->nlights = 1;
        c->node->lights = c->node;
        c->idx = i;
        set1_KPTreeGrid (&kpgrid, i, &c->node->iatt.origin);
        c->remidx = i;
        remlights.s[i] = i;
    } BLose()

    init_KPTree (&kptree);
    build_KPTree (&kptree, &kpgrid);

    {
        Point diag;
        measure_BBox (&diag, &kpgrid.box);
            /* scale_Point (&diag, &diag, 1.0/16); */
        sdsim2 = mag2_Point (&diag);
    }

    while (remlights.sz > 1)
    {
        LightCutBuild close;
        LightCutBuild* c0 = 0;
        LightCutNode* node;

        c0 = &clusters[remlights.s[uint_GMRand (gmrand, remlights.sz)]];

        close = *c0;
        close.node = 0;

        {
            real e0;
            e0 = find_light_match (&close, c0, clusters, &kptree,
                                   Max_real, sdsim2);
            while (true)
            {
                LightCutBuild close_back = close;
                real e1;
                Claim( close.node );
                e1 = find_light_match (&close_back, &clusters[close.idx],
                                       clusters, &kptree, e0, sdsim2);
                if (e0 <= e1)  break;
                Claim( close_back.node );
                e0 = e1;
                c0 = &clusters[close.idx];
                close = close_back;
            }
        }
        Claim( close.node );

        node = &t->nodes.s[nnodes++];
        c0->node->bst.joint = &node->bst;
        close.node->bst.joint = &node->bst;
        *node = *c0->node;
            /* Probabilistically choose a representative based on intensity.*/
        {
            real a = maxmag_Color (&node->color);
            real b = maxmag_Color (&close.node->color);
            if (real_GMRand (gmrand) * (a + b) >= a)
                node->iatt = close.node->iatt;
        }

        node->bst.joint = &t->sentinel.bst;
        node->bst.split[0] = &c0->node->bst;
        node->bst.split[1] = &close.node->bst;
        summ_Color (&node->color, &node->color, &close.node->color);
        c0->box = close.box;
        c0->dox = close.dox;
        c0->cos_bound = node->cos_bound = close.cos_bound;
        node->area += close.node->area;
        node->bbox = close.box;

            /* Shrink /remlights/ by removing the
             * position of the close node.
             */
        remlights.s[close.remidx] = remlights.s[-- remlights.sz];
        clusters[remlights.s[close.remidx]].remidx = close.remidx;

        c0->node = node;
        clusters[close.idx].node = 0;
        clusters[close.idx].idx = c0->idx;
    }

    root_for_BSTree (&t->bst, &clusters[remlights.s[0]].node->bst);

    Claim2( nnodes ,==, 2*nlights-1 );
    Claim2( remlights.sz ,==, 1 );

    lose_KPTreeGrid (&kpgrid);
    lose_KPTree (&kptree);
    LoseTable( uint, remlights );

    linearize_LightCutTree (t);
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

    tree->area = 0;
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
            tree->area += area_Simplex (elem->simplex);
            normalize_Point (&elem->normal,
                             &object->simplices[ei].plane.normal);
        }
    } BLose()

    if (elems.sz == 0)  return;

    {
        const uint neach = nphotons / elems.sz;
        const uint nplus1 = nphotons % elems.sz;

        { BLoop( ei, elems.sz )
            EmisElem* elem = &elems.s[ei];
            elem->nphotons = neach + (ei < nplus1 ? 1 : 0);
            if (elem->nphotons > 0)
                elem->area = area_Simplex (elem->simplex) / elem->nphotons;
            else
                elem->area = 0;
        } BLose()
    }


    {   uint elem_idx = 0;
        uint ephoton_idx = 0;
    { BLoop( photon_idx, nphotons )

        const EmisElem* elem = &elems.s[elem_idx];
        Simplex simplex;
        Color color;
        Ray ray;
        Point c;
        real x;
        real area;

        if (ephoton_idx >= elem->nphotons)
        {
            ephoton_idx = 0;
            do
            {
                ++ elem_idx;
                Claim2( elem_idx ,<, elems.sz );
                elem = &elems.s[elem_idx];
            } while (elem->nphotons == 0);
        }
        ++ ephoton_idx;

        simplex = *elem->simplex;

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
        area = elem->area;
        scale_Color (&color, &color, area / M_PI);

        { BLoop( bounce_idx, nbounces )
            DeclGrow1Table( LightCutNode, lights, light );
            PointXfrm A;
            real zenith = asin (sqrt (real_GMRand (&gmrand)));
            real azimuthcc = 2 * M_PI * real_GMRand (&gmrand);
                /* real zenith = asin (sqrt (halton (2, light_idx))); */
                /* real azimuthcc = 2 * M_PI * halton (3, light_idx); */
            uint hit = Max_uint, obj = Max_uint;
            real mag = Max_real;

            light->color = color;
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
                normalize_Point (&normal, &normal);
                dot = dot_Point (&normal, &ray.direct);

                prod_Color (&color, &color, &matl->diffuse);
                    /* Not sure why to comment...*/
                    /* scale_Color (&color, &color, - dot); */
                if (maxmag_Color (&color) <= Epsilon_real)  break;

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

    } BLose() }

    PackTable( LightCutNode, lights );

    { BLoop( i, space->nlights )
        space->lights[i].on = false;
    } BLose()

    tree->nodes = lights;
    make_light_tree (tree, &gmrand);
    LoseTable( EmisElem, elems );
}

