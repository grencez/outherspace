
#ifndef Radiosity_H_
#define Radiosity_H_

#include "color.h"
#include "gmrand.h"
#include "raytrace.h"
#include "simplex.h"
#include "space.h"
#include "cx/bstree.h"
#include "cx/table.h"
    /* #include "hldseq.h" */

typedef struct LightCutNode LightCutNode;
typedef struct LightCutTree LightCutTree;

struct LightCutNode
{
    BSTNode bst;
    Ray iatt;  /* location + orientation */
    Color color;
};
DeclTableT( LightCutNode, LightCutNode );

struct LightCutTree
{
    BSTNode sentinel;
    BSTree bst;
    Table( LightCutNode ) nodes;
};



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

static inline
    void
cast_lights (RaySpace* space, uint nphotons, uint nbounces)
{
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

    space->nlights = lights.sz;
    ResizeT( PointLightSource, space->lights, space->nlights );

    { BLoop( light_idx, lights.sz )
        PointLightSource* light = &space->lights[light_idx];
        init_PointLightSource (light);
        light->diffuse = true;
        light->hemisphere = true;
        light->location = lights.s[light_idx].iatt.origin;
        light->direct = lights.s[light_idx].iatt.direct;
        light->intensity = lights.s[light_idx].color;
    } BLose()
    LoseTable( LightCutNode, lights );
    LoseTable( EmisElem, elems );
}



#endif

