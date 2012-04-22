
#ifndef __OPENCL_VERSION__
#include "bbox.h"

#include "point.h"
#include "xfrm.h"

#include <assert.h>
#include <math.h>
#include <string.h>
#endif  /* #ifndef __OPENCL_VERSION__ */

    void
zero_BBox (BBox* box)
{
    zero_Point (&box->min);
    zero_Point (&box->max);
}

tristate facing_BoundingPlane (uint dim, real plane,
                               const Point* origin, const Point* dir)
{
    tristate sign;
    sign = compare_real (plane, origin->coords[dim]);
    return mul_signum (sign, signum_real (dir->coords[dim]));
}

    /* Assume ray is coming from inside BBox.*/
    bool
hit_inner_BoundingPlane (Point* entrance,
                         uint dim, real plane,
                         __global const BBox* box,
                         const Point* origin,
                         const Point* dir)
{
    uint i;
    bool didhit = true;
    real coeff;
    tristate facing;

    facing = facing_BoundingPlane (dim, plane, origin, dir);

    if (facing < 0)  return false;

    if (facing == 0)
    {
            /* It's on plane.*/
        UFor( i, NDimensions )
        {
            if (i == dim)
            {
                entrance->coords[i] = origin->coords[i];
            }
            else if (origin->coords[i] < box->min.coords[i])
            {
                if (dir->coords[i] <= 0)  didhit = false;
                else  entrance->coords[i] = box->min.coords[i];
            }
            else if (origin->coords[i] > box->max.coords[i])
            {
                if (dir->coords[i] >= 0)  didhit = false;
                else  entrance->coords[i] = box->max.coords[i];
            }
            else
            {
                entrance->coords[i] = origin->coords[i];
            }
        }
        return didhit;
    }

    coeff = (plane - origin->coords[dim]) / dir->coords[dim];

    UFor( i, NDimensions )
    {
        real x;
        if (i == dim)
        {
            x = plane;
        }
        else
        {
            x = origin->coords[i] + coeff * dir->coords[i];
            if (x < box->min.coords[i])
            {
                if (dir->coords[i] < 0)  didhit = false;
                else  x = box->min.coords[i];
            }
            else if (x > box->max.coords[i])
            {
                if (dir->coords[i] > 0)  didhit = false;
                else  x = box->max.coords[i];
            }
        }
        entrance->coords[i] = x;
    }
    return didhit;
}

    real
hit_inner_BBox (Point* isect,
                uint* ret_dim,
                const BBox* box,
                const Ray* ray,
                const Point* invdirect)
{
    uint dim, hit_dim;
    real planes[NDimensions];
    real mags[NDimensions];
    const Point* origin;
    origin = &ray->origin;
    UFor( dim, NDimensions )
    {
        real x;
        x = invdirect->coords[dim];
        if (x < 0)
        {
            planes[dim] = box->min.coords[dim];
            mags[dim] = (planes[dim] - origin->coords[dim]) * x;
        }
        else if (x > 0)
        {
            planes[dim] = box->max.coords[dim];
            mags[dim] = (planes[dim] - origin->coords[dim]) * x;
        }
        else
        {
            planes[dim] = box->max.coords[dim];
            if (origin->coords[dim] == planes[dim])
                mags[dim] = 0;
            else
                mags[dim] = Max_real;
        }
    }

    hit_dim = NDimensions-1;

    UFor( dim, NDimensions-1 )
        if (mags[dim] < mags[hit_dim])
            hit_dim = dim;

    follow_Point (isect, &ray->origin, &ray->direct, mags[hit_dim]);

    isect->coords[hit_dim] = planes[hit_dim];
    *ret_dim = hit_dim;
    return mags[hit_dim];
}

    /* Assume ray is coming from outside BBox.*/
bool hit_outer_BBox (Point* entrance,
                     __global const BBox* box,
                     const Point* origin, const Point* dir)
{
    uint dim;
    Point cost;
    real hit_cost;
    uint hit_dim;

    UFor( dim, NDimensions )
    {
        if (dir->coords[dim] > 0)
        {
            if (origin->coords[dim] > box->max.coords[dim])
                return false;
            if (origin->coords[dim] > box->min.coords[dim])
                cost.coords[dim] = Small_real;
            else
                cost.coords[dim] = ((box->min.coords[dim]
                                     - origin->coords[dim])
                                    / dir->coords[dim]);
        }
        else if (dir->coords[dim] < 0)
        {
            if (origin->coords[dim] < box->min.coords[dim])
                return false;
            if (origin->coords[dim] < box->max.coords[dim])
                cost.coords[dim] = Small_real;
            else
                cost.coords[dim] = ((box->max.coords[dim]
                                     - origin->coords[dim])
                                    / dir->coords[dim]);
        }
        else
        {
            cost.coords[dim] = Small_real;
        }
    }

    hit_cost = 0;
    hit_dim = 0;

    UFor( dim, NDimensions )
    {
        if (cost.coords[dim] > hit_cost)
        {
            hit_cost = cost.coords[dim];
            hit_dim = dim;
        }
    }

    UFor( dim, NDimensions )
    {
        if (dim == hit_dim)
        {
            if (dir->coords[dim] > 0)
                entrance->coords[dim] = box->min.coords[dim];
            else
                entrance->coords[dim] = box->max.coords[dim];
        }
        else
        {
            entrance->coords[dim] = (origin->coords[dim]
                                     + hit_cost * dir->coords[dim]);
            if (dir->coords[dim] > 0)
            {
                if (entrance->coords[dim] > box->max.coords[dim])
                    return false;
            }
            else
            {
                if (entrance->coords[dim] < box->min.coords[dim])
                    return false;
            }
        }
    }

    return true;
}

bool hit_BBox (Point* entrance,
               const BBox* box,
               const Point* origin, const Point* dir)
{
    uint dim;
    Point cost;
    real hit_cost;

    UFor( dim, NDimensions )
    {
        if (dir->coords[dim] > 0)
        {
            real bound;
            if (origin->coords[dim] > box->max.coords[dim])
                return false;
            if (origin->coords[dim] > box->min.coords[dim])
                bound = box->max.coords[dim];
            else
                bound = box->min.coords[dim];

            cost.coords[dim] = ((bound - origin->coords[dim])
                                / dir->coords[dim]);
        }
        else if (dir->coords[dim] < 0)
        {
            real bound;
            if (origin->coords[dim] < box->min.coords[dim])
                return false;
            if (origin->coords[dim] < box->max.coords[dim])
                bound = box->min.coords[dim];
            else
                bound = box->max.coords[dim];

            cost.coords[dim] = ((bound - origin->coords[dim])
                                / dir->coords[dim]);
        }
        else
        {
            cost.coords[dim] = Max_real;
        }
    }

    hit_cost = Max_real;

    UFor( dim, NDimensions )
        if (cost.coords[dim] < hit_cost)
            hit_cost = cost.coords[dim];

    UFor( dim, NDimensions )
        entrance->coords[dim] = (origin->coords[dim]
                                 + hit_cost * dir->coords[dim]);
    return true;
}

    void
init0_BBox (BBox* box)
{
    set_Point (&box->min, Max_real);
    set_Point (&box->max, Min_real);
}

void init_BBox (BBox* box, uint npoints, const Point* points)
{
    uint i;

    if (npoints == 0)
    {
        zero_Point (&box->min);
        zero_Point (&box->max);
    }
    else
    {
        copy_Point (&box->min, &points[0]);
        copy_Point (&box->max, &points[0]);
    }

    UFor( i, npoints )
        adjust_BBox (box, &points[i]);
}

void adjust_BBox (BBox* box, const Point* point)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (point->coords[i] < box->min.coords[i])
            box->min.coords[i] = point->coords[i];
        else if (point->coords[i] > box->max.coords[i])
            box->max.coords[i] = point->coords[i];
    }
}

    void
include_BBox (BBox* dst, const BBox* a, const BBox* b)
{
    uint i;
    UFor( i, NDimensions )
    {
        dst->min.coords[i] = ((a->min.coords[i] <= b->min.coords[i])
                              ? a->min.coords[i] : b->min.coords[i]);
        dst->max.coords[i] = ((a->max.coords[i] >= b->max.coords[i])
                              ? a->max.coords[i] : b->max.coords[i]);
    }
}

    void
centroid_BBox (Point* dst, const BBox* box)
{
    summ_Point (dst, &box->min, &box->max);
    scale_Point (dst, dst, .5);
}

    void
measure_BBox (Point* dst, const BBox* box)
{
    diff_Point (dst, &box->max, &box->min);
}

bool inside_BBox (__global const BBox* box, const Point* point)
{
    bool inside = true;
    uint i;
    UFor( i, NDimensions )
    {
        if (!(box->min.coords[i] <= point->coords[i] &&
              box->max.coords[i] >= point->coords[i]))
            inside = false;
    }
    return inside;
}

real surface_area_BBox (const BBox* box)
{
    Point delta;
    real sum = 0;
    uint i;

    diff_Point (&delta, &box->max, &box->min);

    UFor( i, NDimensions )
    {
        real prod = 1;
        uint j;
        UFor( j, NDimensions )
        {
            if (i != j)
            {
                prod *= delta.coords[j];
            }
        }
        sum += prod;
    }
    return 2 * sum;
}

void split_BBox (BBox* lo_box, BBox* hi_box,
                 const BBox* box,
                 uint split_dim, real split_pos)
{
    assert (split_pos <= box->max.coords[split_dim]);
    assert (split_pos >= box->min.coords[split_dim]);

    *lo_box = *box;
    *hi_box = *box;

    lo_box->max.coords[split_dim] = split_pos;
    hi_box->min.coords[split_dim] = split_pos;
}

    void
merge_BBox (BBox* dst, const BBox* a, const BBox* b)
{
    uint i;
    UFor( i, NDimensions )
    {
        if (a->min.coords[i] <= b->min.coords[i])
            dst->min.coords[i] = a->min.coords[i];
        else
            dst->min.coords[i] = b->min.coords[i];

        if (a->max.coords[i] >= b->max.coords[i])
            dst->max.coords[i] = a->max.coords[i];
        else
            dst->max.coords[i] = b->max.coords[i];
    }
}

    void
clip_BBox (BBox* dst, const BBox* a, const BBox* b)
{
    uint i;
    UFor( i, NDimensions )
    {
            /* TODO  Why does this fail sometimes? */
#if 0
        assert (a->min.coords[i] <= b->max.coords[i]);
#else
        if (a->min.coords[i] > b->max.coords[i])
            dst->min.coords[i] = b->max.coords[i];
        else
#endif
        if (a->min.coords[i] >= b->min.coords[i])
            dst->min.coords[i] = a->min.coords[i];
        else
            dst->min.coords[i] = b->min.coords[i];

            /* TODO  Why does this fail sometimes? */
#if 0
        assert (a->max.coords[i] >= b->min.coords[i]);
#else
        if (a->max.coords[i] < b->min.coords[i])
            dst->min.coords[i] = b->min.coords[i];
        else
#endif
        if (a->max.coords[i] <= b->max.coords[i])
            dst->max.coords[i] = a->max.coords[i];
        else
            dst->max.coords[i] = b->max.coords[i];
    }
}


    /** Take a bounding box, rotate it from a basis,
     * and reposition it around a new centroid.
     * The bounding box volume will grow in most cases,
     * as it is axis aligned and the axes have changed!
     * Note: View /new_centroid/ as a displacement.
     **/
    void
trxfrm_BBox (BBox* dst,
             const PointXfrm* basis,
             const BBox* box,
             const Point* new_centroid)
{
    uint i;
    Point diff;
    PointXfrm bbox, robox;

    Op_1200( real, NDims, diff.coords
             ,.5*, -, box->max.coords , box->min.coords );

        /* Fill rows in /bbox/ matrix with components of /diff/,
         * flipping signs of the components based on the signs
         * of the basis matrix's corresponding columns.
         */
    UFor( i, NDimensions )
    {
        uint j;
        UFor( j, NDimensions )
        {
            if (basis->pts[j].coords[i] >= 0)
                bbox.pts[i].coords[j] = + diff.coords[j];
            else
                bbox.pts[i].coords[j] = - diff.coords[j];
        }
    }

        /*    T   T T
         *  (B * A )  = A * B
         */
    xfrm_PointXfrm (&robox, &bbox, basis);

        /* Set /diff/ to be the displacement,
         * taking into account the original bbox centroid.
         */
    centroid_BBox (&diff, box);
    trxfrm_Point (&diff, basis, &diff);
    summ_Point (&diff, &diff, new_centroid);

    UFor( i, NDimensions )
    {
        uint j;
        real a, b;
        a = robox.pts[i].coords[i];
        b = diff.coords[i];
        dst->min.coords[i] = - a + b;
        dst->max.coords[i] = + a + b;

        UFor( j, NDimensions )
                /* assert (a >= robox.pts[j].coords[i]); */
            assert (relative_error (a, robox.pts[j].coords[i], 1)
                    <= 2*Epsilon_real);
    }
}

