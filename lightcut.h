
#ifndef LIGHTCUT_H_
#define LIGHTCUT_H_

#include "color.h"
#include "point.h"
#include "raytrace.h"
#include "simplex.h"
#include "xfrm.h"
#include "util.h"
    /* #include "hldseq.h" */


void
init_LightCutTree (LightCutTree* t);
void
lose_LightCutTree (LightCutTree* t);
void
cast_lights (RaySpace* space, uint nphotons, uint nbounces);

qual_inline
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
            /* fprintf (stderr, "wah:%f %f\n", elo, ehi); */
        a = a->split[0];
    }

    return CastUp( LightCutNode, bst, a );
}

qual_inline
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

#if 0
    for (node = &tree->nodes.s[0];
         node != &tree->nodes.s[nlights];
         ++ node)
#else
    while ((node = next_LightCutNode (tree, node, 1.0 / 256, hit)))
#endif
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
            real c = 1 / node->area;
            real dist_factor = 1 / (magtolight * magtolight);
            tscale *= dist_factor;
            if (tscale > c)  tscale = c;
                /* Add diffuse portion.*/
            follow_Color (&color, &color, &node->color, tscale);
        }

    }
    prod_Color (ret_color, &color, diffuse);
        /* fprintf (stderr, "%u\n", ni); */
}

#ifdef IncludeC
#include "lightcut.c"
#endif
#endif

